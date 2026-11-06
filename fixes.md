# Правки скрипта Abaqus: заклёпочное соединение (main.py)

Нужно внести правки в существующий `main.py` (Abaqus 2018 Student Edition, запуск `abaqus cae noGUI=main.py`, Python 2.7). Структуру файла, имена переменных и логику, не упомянутую ниже, не менять. После правок убедиться, что файл синтаксически корректен для Python 2.7.

## Контекст задачи

- Прямоугольная 2D пластина (`TWO_D_PLANAR`, деформируемая): AB = 150 мм, BC = 50 мм, толщина 5 мм.
- Два отверстия R = 10 мм, центры в (30; 25) и (120; 25) мм.
- Две заклёпки: внешний радиус 10 мм, внутреннее отверстие R = 4 мм, толщина 5 мм.
- Материал: E = 200·10⁹ Па = 200000 МПа (система мм-Н-МПа), ν = 0.3.
- Контакт заклёпка–пластина: **без трения**, жёсткий нормальный контакт.
- Анализ: Static (базовый шаг) + Static, Linear Perturbation (шаг с нагрузкой).
- ГУ: u1 = u2 = ur3 = 0 по внутренним отверстиям заклёпок.
- Нагрузка: равномерно распределённая на верхней грани AB, 1000 Н/мм².
- Элемент: CPS4R. Лимит Student Edition: 1000 узлов.
- Точка E = (75; 25) мм, пересечение оси симметрии и линии центров отверстий. Требуется найти напряжение в ней.

## Правка 1. Модуль Юнга

Было `YOUNG_MODULUS = 200.0` (занижен в 1000 раз). Сделать:

```python
YOUNG_MODULUS = 200000.0   # 200e9 Pa = 200000 MPa
```

## Правка 2. Контакт без трения

В `create_model()` заменить:

```python
cp.TangentialBehavior(formulation=ROUGH)
```

на:

```python
cp.TangentialBehavior(formulation=FRICTIONLESS)
```

## Правка 3. Явные контактные пары вместо general contact

Удалить весь блок общего контакта:

```python
model.ContactStd(name='GeneralContact', createStepName='Initial')
model.interactions['GeneralContact'].includedPairs.setValuesInStep(...)
model.interactions['GeneralContact'].contactPropertyAssignments.appendInStep(...)
```

На его место вставить явные пары «внешний край заклёпки — край отверстия пластины» для обеих заклёпок. Вставлять **после** создания экземпляров и их трансляции, после блока `ContactProperty`.

В начало файла (рядом с остальными импортами) добавить `from math import hypot`.

В раздел `HELPERS` добавить функцию:

```python
def ring_edges(inst, cx, cy, r):
    """Рёбра экземпляра inst, лежащие на окружности радиуса r с центром (cx, cy)."""
    pts = []
    for e in inst.edges:
        p = e.pointOn[0]
        if abs(hypot(p[0] - cx, p[1] - cy) - r) < 0.5:
            pts.append(p)
    return inst.edges.findAt(coordinates=tuple(pts))
```

Код создания пар (внутри `create_model()`):

```python
for tag, inst, cx, cy in (('L', rivet_left_inst,  HOLE_LEFT_X,  HOLE_LEFT_Y),
                          ('R', rivet_right_inst, HOLE_RIGHT_X, HOLE_RIGHT_Y)):
    a.Surface(name='RivetOuter_' + tag,
              side1Edges=ring_edges(inst, cx, cy, RIVET_OUTER_RADIUS))
    a.Surface(name='PlateHole_' + tag,
              side1Edges=ring_edges(plate_inst, cx, cy, HOLE_RADIUS))
    model.SurfaceToSurfaceContactStd(
        name='Contact-' + tag, createStepName='Initial',
        master=a.surfaces['RivetOuter_' + tag],
        slave=a.surfaces['PlateHole_' + tag],
        sliding=SMALL,
        interactionProperty='RivetPlateContact')
```

Важно: пары создаются в шаге `Initial`, чтобы они были активны в обоих шагах. Внутренние рёбра заклёпки (R = 4 мм) в поверхность не попадают, так как отбор идёт по радиусу.

## Правка 4. Предварительная нагрузка в базовом шаге

Линейно-возмущённый шаг использует состояние контакта из конца базового шага. В `BaseStep` нет нагрузки, и пластина удерживается только контактом без трения. Это даёт предупреждения о жёстком смещении и неопределённое состояние контакта.

После создания поверхности `PlateTopSurface` и существующей `TopPressure` добавить малую предварительную нагрузку в `BaseStep`:

```python
model.Pressure(name='PreLoad', createStepName='BaseStep',
               region=a.surfaces['PlateTopSurface'], magnitude=1.0)
```

`TopPressure` в `LoadStep` (magnitude = `PRESSURE`) оставить без изменений.

## Правка 5. Чтение напряжений в узлах (самая важная для результата)

CPS4R по умолчанию отдаёт `S` в точках интегрирования, поэтому `val.nodeLabel` равен `None` и в `extract_results()` напряжение в узле не находится. Нужно запрашивать значения в узлах элементов и усреднять по элементам, которые делят узел.

В `extract_results()` заменить:

```python
s_field = frame.fieldOutputs['S'].getSubset(region=inst)
```

на:

```python
s_field = frame.fieldOutputs['S'].getSubset(region=inst,
                                            position=ELEMENT_NODAL)
```

Строку `u_field = frame.fieldOutputs['U'].getSubset(region=inst)` оставить без изменений.

Блок, где ищется `stress_at_e`:

```python
for val in s_field.values:
    if val.nodeLabel == closest_node_label:
        stress_at_e = val
        break
```

заменить на усреднение:

```python
s_vals = [v for v in s_field.values if v.nodeLabel == closest_node_label]
if s_vals:
    n = float(len(s_vals))
    s11 = sum(v.data[0] for v in s_vals) / n
    s22 = sum(v.data[1] for v in s_vals) / n
    s33 = sum(v.data[2] for v in s_vals) / n
    s12 = sum(v.data[3] for v in s_vals) / n
    mises = sum(v.mises for v in s_vals) / n
    stress_at_e = True
else:
    stress_at_e = None
```

В блоке записи `summary.txt` вместо `stress_at_e.data[0]`, `stress_at_e.data[1]`, `stress_at_e.data[2]`, `stress_at_e.data[3]`, `stress_at_e.mises` использовать переменные `s11`, `s22`, `s33`, `s12`, `mises`. Индексы компонент в ODB для плоского напряжённого состояния: `data[0]` = S11, `data[1]` = S22, `data[2]` = S33 (равно 0), `data[3]` = S12. Это верно, менять нумерацию не нужно.

В `summary.txt` дополнительно записать расстояние от ближайшего узла до точки E (значение `min_dist ** 0.5`), чтобы было видно точность привязки.

## Правка 6. Узел точно в точке E (желательно)

При сетке 4 мм узла ровно в (75; 25) может не быть. Чтобы он был, до `generateMesh()` разделить пластину партицией по оси симметрии x = 75 мм:

```python
dp = plate.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=POINT_E_X)
plate.PartitionFaceByDatumPlane(datumPlane=plate.datums[dp.id],
                                faces=plate.faces[:])
```

После этого проверить, что число узлов остаётся меньше 1000 (проверка уже есть в скрипте). При необходимости увеличить `MESH_SIZE` до 4.5–5.0.

## Правка 7. Очистка

- Удалить неиспользуемые `import csv`, `EXPORT_PNG`, `EXPORT_CSV`.
- В `summary.txt` строку `Contact: General (rivets on plate holes)` заменить на `Contact: frictionless, surface-to-surface (rivet outer edge vs plate hole)`.

## Чего не менять

- Арки `Arc3Points` для окружностей. Две дуги дают корректный замкнутый контур, отверстия вырезаются.
- Запрос `F-Output-1`. Он создаётся автоматически и наследуется шагами.
- Имя экземпляра `'PLATE-1'` в ODB (верхний регистр верный).
- Граничные условия на внутренние края заклёпок (u1 = u2 = ur3 = 0).

## Примечания для отчёта (в код не вносить)

- Давление 1000 Н/мм² даст напряжения далеко за пределом текучести стали. Для линейной задачи это допустимо, но стоит отметить в выводах.
- В условии есть расхождение: «давление 1000 Н» в описании и «σ = 1000 Н/мм²» в разделе нагрузки. Скрипт следует варианту с Н/мм².

## Проверка после правок

1. Скрипт запускается без ошибок: `abaqus cae noGUI=main.py`.
2. В `.msg` нет предупреждений о жёстком смещении (rigid body motion) и неконвергирующем контакте.
3. Число узлов меньше 1000.
4. В `summary.txt` есть напряжения S11, S22, S12 и Mises в точке E, а также расстояние до ближайшего узла.
5. По ODB контакт замкнут в верхней половине каждого отверстия (`CPRESS` > 0 на поверхностях `PlateHole_L` и `PlateHole_R`). При желании добавить `CPRESS` в `fieldOutputRequests`.
