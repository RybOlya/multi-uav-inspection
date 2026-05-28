# Сцена для Isaac Sim: склад Pegasus або власний USD

Координатна система, з якою працюють усі команди оператора:

| Вісь | Зміст |
|------|--------|
| **X**, **Y** | Метри в горизонтальній площині сцени |
| **Z** | Підлога інспекції на висоті `Z = 0` |

Висоти, які пишете в команди (`--altitude`, `--z`) — це метри над підлогою.
Завжди тримайте їх із запасом 1–2 м під найнижчим елементом стелі / ферми.

---

## Режими (`.env` або змінні середовища)

| Змінна | Значення | Пояснення |
|--------|----------|-----------|
| `SIM_SCENE_STYLE` | `warehouse` (за замовчуванням) | Готова сцена з `pegasus.simulator.params.SIMULATION_ENVIRONMENTS`, ім'я задається в `SIM_PEGASUS_ENV_NAME`. Завантажується через NVIDIA Nucleus. |
| | `usd` | Власний файл `.usd`/`.usda`/`.usdc` із диску |
| `SIM_PEGASUS_ENV_NAME` | `Full Warehouse` | Інші доступні: `Default Environment`, `Curved Gridroom`, `Hospital`, `Office`, `Simple Room`, `Warehouse`, `Warehouse with Forklifts`, `Warehouse with Shelves`, `Flat Plane` |
| `SIM_USD_SCENE_PATH` | абсолютний шлях | Для `scene_style=usd` |

Якщо Nucleus недоступний (`connection refused` / `cannot resolve omniverse://`),
код автоматично відкочується на дефолтну Isaac Sim площину з підлогою —
симуляція все одно стартує, ви просто отримаєте порожній простір.

---

## Варіант A — готовий склад Pegasus (рекомендовано)

Це дефолт. Pegasus при старті викликає
`pg.load_environment(SIMULATION_ENVIRONMENTS["Full Warehouse"])` і підвантажує
сцену з NVIDIA Nucleus. Жодних змін у `.env` не потрібно.

Якщо хочете іншу заводську сцену — змініть `SIM_PEGASUS_ENV_NAME`:

```bash
SIM_SCENE_STYLE=warehouse
SIM_PEGASUS_ENV_NAME="Warehouse with Shelves"   # або "Warehouse with Forklifts"
```

---

## Варіант B — власний USD (Blender / Omniverse Composer)

1. **Створити геометрію** простого об'єму (підлога, стіни за бажанням).
2. **Вирівняти підлогу з площиною `Z = 0`** — у Blender застосуйте
   `Apply Transforms`, опустіть меш так, щоб верхній рівень підлоги був
   на `Z = 0`.
3. **Експорт USD** — з Omniverse Composer: `File → Export → USD`;
   з Blender: офіційний USD-аддон або експорт у `.glb`/`.fbx` і конвертація
   через Composer.
4. **Перший запуск:**

   ```bash
   SIM_SCENE_STYLE=usd \
   SIM_USD_SCENE_PATH=/повний/шлях/factory.usd \
       ./python.sh /шлях/до/bridge-swarm-iot/isaac/swarm_app.py
   ```

5. **Якщо дрони "під землею" або за стінами** — додайте порожній Xform-
   батько над root і скоригуйте `translate`. Команди оператора (`takeoff`,
   `goto`, `scan`) використовують абсолютні координати сцени, тож
   простежте, щоб ваш робочий простір лежав там, де ви очікуєте у `+X/+Y`.
