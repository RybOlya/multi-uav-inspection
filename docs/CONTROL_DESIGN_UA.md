# Архітектура керування рою — обґрунтування за літературою

> Пояснювальна записка має посилатись лише на доведені, цитовані компоненти.
> Усе нижче спирається на роботи [4], [9], [17], [22], [24], [25], [26]
> (повні назви — наприкінці документа). Власних незалежних модулів,
> які не мають літературного джерела, в системі немає.

## 1. Стек керування

Архітектура — чотирирівнева ієрархія *operator – planner – tracker – plant*:

```
Operator CLI (swarm.operator)
        │  JSON command  ─── MQTT v5 [17] ───►
        ▼
swarm.commands (Pydantic)
        ▼
swarm.controller.CommandedNonlinearController
        │
        ├── swarm.trajectory.TrajectoryFollower
        │     ├── single quintic segment   ── Lynch & Park 2017 §9.2 [26]
        │     │       (takeoff, goto, hover, land)
        │     └── chained quintic segments ── Lynch & Park 2017 [26]
        │             through Boustrophedon waypoints (Choset 2000 [4])
        │
        └── pegasus NonlinearController (без модифікацій)
                ├── outer-loop position PD + I            ─┐
                ├── SO(3) attitude PD                       ├─ Mellinger & Kumar 2011 [25]
                └── force / torque → rotor allocation       │  Pinto, Guerreiro, Cunha 2021 [24]
                                                            │  (теоретичний анкер: Lee, Leok,
                                                            │   McClamroch 2010 [9])
                                                            ▼
                                                pegasus Multirotor (Isaac Sim PhysX) [22]
```

Одна `CommandedNonlinearController` на дрон. Усі дрони незалежні: жодного
між-дронового обміну командами немає, бо **жодне з ужитих джерел не вимагає
цього для виконання задачі типу "оператор → дрон → точка / зона"**.

## 2. Внутрішній контур: `pegasus.NonlinearController`

Це готовий клас із прикладу `examples/utils/nonlinear_controller.py`
дистрибутиву Pegasus Simulator [22]. Він реалізує:

* PD з I-членом по позиції в інерціальній рамці:
  \(F_{\text{des}} = -K_p e_p - K_d e_v - K_i \int e_p\,dt + m g \mathbf e_3 + m a_{\text{ref}}\)
* Витягання необхідного скаляру тяги \(u_1 = F_{\text{des}}^\top R\,\mathbf e_3\)
* Побудову бажаної базисної трійки тіла \([X^b_d, Y^b_d, Z^b_d]\) з вектора
  \(F_{\text{des}}\) і референсного yaw → \(R_d\).
* Помилку обертання \(e_R = \tfrac12 (R_d^\top R - R^\top R_d)^{\vee}\) і
  бажану кутову швидкість через jerk-канал
  \(h_w = (m/u_1)\bigl(j_{\text{ref}} - (Z^b_d \cdot j_{\text{ref}})\,Z^b_d\bigr)\).
* Моменти \(\tau = -K_R e_R - K_w e_w\).
* Алокацію \((u_1, \tau) \to (\omega_1,\dots,\omega_4)\) штатною матрицею
  Pegasus' `Multirotor.force_and_torques_to_velocities`.

Це **дослівно** алгоритм Mellinger & Kumar 2011 [25] із розширенням Pinto,
Guerreiro, Cunha 2021 [24]; математично еквівалентний геометричному
SE(3)-трекеру Lee, Leok, McClamroch 2010 [9].

**Ми не модифікуємо математику.** Підкласовий `CommandedNonlinearController`
у `swarm/controller.py` лише переписує блок, що читає референс із CSV: замість
рядка таблиці він викликає `TrajectoryFollower.advance(dt)` і отримує
\((p_{\text{ref}}, v_{\text{ref}}, a_{\text{ref}}, j_{\text{ref}},
\psi_{\text{ref}}, \dot\psi_{\text{ref}})\) у тому самому форматі, який
очікує батьківська ``update``. Усі коефіцієнти \(K_p, K_d, K_i, K_R, K_w\) і
параметри \(m, g\) — за замовчуваннями [22].

## 3. Генератор траєкторій: квінтична поліноміальна часова шкала

Lynch & Park 2017, §9.2 [26], стверджує що **п'ятого** порядку поліном —
найнижчий, який одночасно задовольняє граничним умовам по позиції,
швидкості та прискоренню в обох кінцях. Менший порядок (кубічний) залишає
стрибок прискорення → перетворюється на стрибок моменту через [25, eq. 17],
що неприйнятно для квадрокоптера.

Для одно-сегментних команд (`takeoff`, `goto`, `hover`, `land`) ми будуємо
сегмент із \(v_0=v_1=0,\ a_0=a_1=0\) (рівняння 9.13–9.16 з [26]).
Тривалість \(T = \max(T_{\min},\ \|p_1-p_0\| / v_{\text{cruise}})\) — це
таргетна крейсерська швидкість, що задається `SIM_CRUISE_SPEED_MPS` або
полем `cruise_speed_mps` у конкретній команді.

Для команди `scan` Boustrophedon-послідовність waypoint-ів (див. розд. 4)
з'єднується **chained-сегментами**: на стиках \(v_i\) ≠ 0 (вектор
біссектриси між векторами входу/виходу нормалізовано до \(v_{\text{cruise}}\)),
\(a_i = 0\). Це прагматичне послаблення повного *minimum-snap* QP [25] з
збереженням C¹-неперервності у наданому референсі — достатньо, бо
Pegasus' трекер диференціює тільки до jerk (див. [25, eq. 7]).

## 4. Команда `scan` = одна Boustrophedon-комірка (Choset 2000)

Coverage Path Planning у нашій постановці зводиться до примітиву
**Boustrophedon Cellular Decomposition** (BCD) у його найпростішому
вигляді — однієї монотонної прямокутної комірки [4, §4]. Тобто:

1. Оператор вказує осесиметричний прямокутник \([x_{\min},x_{\max}]\times
   [y_{\min},y_{\max}]\) і висоту \(z\).
2. `swarm.coverage.boustrophedon_path` будує впорядкований список
   waypoint-ів, що "косять" вздовж осі \(x\) (або \(y\) — параметр)
   зі смугами шириною `sweep_spacing_m`. Чергування напрямку на сусідніх
   смугах, як того й вимагає [4].

Декомпозиція складніших полігонів (з перешкодами) — поза скоупом цієї
курсової: вона мала б додатковий пейпер як джерело (наприклад, повне BCD
[4, §3] чи E-GTSP-формулювання — Bähnemann et al. 2019, але ми не йдемо
туди). Оператор сам обирає, який прямокутник просканувати, і це безпечно,
бо: (а) він бачить сцену; (б) у симуляції перешкоди легко переносити
у вільну зону.

## 5. Канал зв'язку: MQTT v5

Згідно з OASIS MQTT 5.0 [17], використовуємо `paho-mqtt` v5 і три
сімейства топіків:

| Напрям | Топік | QoS | Кодування |
|---|---|---|---|
| operator → drone | `swarm/cmd/{drone_id}` | 1 (at-least-once [17, §4.3]) | JSON, схема — `swarm/commands.py` (Pydantic v2) |
| operator → all | `swarm/cmd/all` | 1 | те саме |
| drone → cloud | `swarm/state/{drone_id}` | 0 (at-most-once) | JSON {mode, position, velocity, yaw, …} |
| drone → cloud | `swarm/telemetry/{drone_id}` | 0 | InfluxDB Line Protocol |
| drone → cloud | `swarm/events/{kind}` | 1 | JSON: `command_accepted`, `command_rejected`, `trajectory_finished`, `drone_online/offline` |

Розділення *cmd / state / telemetry / events* відповідає принципам
сповіщення з різними рівнями важливості [17, §4.3]: критичні
повідомлення (команди, події) — на QoS 1 з `PUBACK`-handshake;
високочастотна телеметрія — QoS 0, без підтверджень.

## 6. Дрон-сторона: машина станів

`CommandedNonlinearController` має простий стан-машину з режимами:

```
   idle ──takeoff──► takeoff ──finish──► hover
    ▲                  │
    │       goto       ▼
    └─── hover ◄── goto ──finish──► hover
    ▲                  ▲
    │                  │
    │       scan       │
    └─── hover ◄── scan ──finish──► hover
    ▲
    │      land
    └─── hover ──land──► land ──finish──► idle
```

Поточний `mode` публікується у `state`-топік. Коли `TrajectoryFollower`
завершує всі сегменти, контролер автоматично переходить у hover на
останній точці та публікує подію `trajectory_finished`. На будь-якому етапі
команда `stop` (alias `hover`) перериває поточну траєкторію й одразу
підвішує дрон у поточній позиції.

## 7. Локалізація: де реальний VIO

Ми **не реалізовуємо** SLAM. Pegasus' `update_state(state)` подає у
контролер ground-truth позицію/швидкість/орієнтацію з PhysX. Це чесно і
відповідає тому, що в [22] описано як "perfect-state baseline" для
тестування алгоритмів керування у відриві від оцінювача.

Для реального GPS-denied середовища у `update_state` слід підставити
оцінку від справжнього стека VIO:

* **ORB-SLAM3** [7] — first-real-time visual-inertial SLAM з multi-map; на
  стерео-IMU датасеті EuRoC: середня помилка ATE 3.6 см.
* **VINS-Fusion** [8] — оптимізаційний фреймворк локального VIO зі
  стерео-варіантом, baseline проти ORB-SLAM3.

Цей перехід — *архітектурно* нульовий: контролер споживає те саме
``State``, тому єдина зміна — `update_state(...)` тепер отримує
оцінювані, а не справжні величини, і похибка \(e_p, e_v\) обчислюється
проти оцінок.

## 8. Чому НЕ було залишено старе

Чесний перелік того, що було видалено й чому:

| Видалене | Чому не пройшло "доведено в літературі" |
|---|---|
| `SimpleSLAM` (експоненційний фільтр + псевдо-loop-closure) | Не цитований метод; не реалізує жодного з відомих алгоритмів VIO. Поведінкова модель, не оцінювач. |
| `SwarmCoordinator` (поділ області смугами через `np.linspace`) | Не цитована схема; на відміну від справжніх decentralized coverage methods (Lloyd / Voronoi gossip [Durham et al. 2010] чи GM-VPC), не має формальних гарантій. |
| `defect_detector` (псевдовипадкове "виявлення дефектів") | Не модель сприйняття. Замінюється YOLOv8 / SAM на окремих кадрах при потребі — поза скоупом курсової. |
| `step_kinematic` / `XFormPrim.set_world_pose` | Це не контроль, а "телепортація" в обхід PhysX. У літературі такого не існує як закону керування. |
| 5 ad-hoc патернів (`u_shape`, `zigzag`, `spiral`, `grid`) | Усі ad-hoc; залишена тільки одна дійсна примітивна — Boustrophedon-sweep [4]. |

## 9. Що дозволяє зробити поточна система

* **Запускати** симуляцію (`isaac/swarm_app.py`) → N дронів спавняться
  у Pegasus-сцені й висять на місці.
* **Командувати** оператором із будь-якого терміналу через CLI:
  `python -m swarm.operator takeoff/goto/scan/hover/stop/land …`.
* **Спостерігати** стан і телеметрію через MQTT (`make subscribe`,
  `make state`).
* **Автоматизувати** командну послідовність — оскільки протокол JSON-MQTT,
  будь-який скрипт або веб-дашборд може видавати команди.

## Список джерел

[4]  Choset, H. *"Coverage of Known Spaces: The Boustrophedon Cellular
     Decomposition"*. **Autonomous Robots** 9, 247–253 (2000).
     <https://doi.org/10.1023/A:1008958800904>

[7]  Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M. M.,
     Tardós, J. D. *"ORB-SLAM3: An Accurate Open-Source Library for Visual,
     Visual-Inertial and Multi-Map SLAM"*. **IEEE T-RO** 37(6), 2021.
     arXiv:2007.11898.

[8]  Qin, T., Pan, J., Cao, S., Shen, S. *"A General Optimization-based
     Framework for Local Odometry Estimation with Multiple Sensors
     (VINS-Fusion)"*, 2019.

[9]  Lee, T., Leok, M., McClamroch, N. H. *"Geometric Tracking Control of
     a Quadrotor UAV on SE(3)"*. **49th IEEE Conference on Decision and
     Control (CDC)**, 5420–5425 (2010). arXiv:1003.2005.

[17] OASIS Standard. *MQTT Version 5.0*, 2019.
     <https://docs.oasis-open.org/mqtt/mqtt/v5.0/mqtt-v5.0.pdf>

[22] Jacinto, M., Pinto, J., Patrikar, J., Keller, J., Cunha, R., Scherer,
     S., Pascoal, A. *"Pegasus Simulator: An Isaac Sim Framework for
     Multiple Aerial Vehicles Simulation"*. **ICUAS 2024**.
     arXiv:2404.15923.

[24] Pinto, J., Guerreiro, B. J., Cunha, R. *"Planning Parcel Relay
     Manoeuvres for Quadrotors"*. **2021 International Conference on
     Unmanned Aircraft Systems (ICUAS)**, 137–145.
     doi:10.1109/ICUAS51884.2021.9476757.

[25] Mellinger, D., Kumar, V. *"Minimum snap trajectory generation and
     control for quadrotors"*. **2011 IEEE International Conference on
     Robotics and Automation (ICRA)**, 2520–2525.
     doi:10.1109/ICRA.2011.5980409.

[26] Lynch, K. M., Park, F. C. *Modern Robotics: Mechanics, Planning, and
     Control*. Cambridge University Press, 2017. §9.2 Point-to-Point
     Trajectories.
