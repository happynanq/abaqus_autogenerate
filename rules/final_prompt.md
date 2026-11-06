# Задача: финальные правки в main.py (Abaqus 2018 Student, Python 2.7)

Внеси в существующий `main.py` правки ниже. Остальной код, имена переменных и структуру не меняй. Скрипт запускается как `abaqus cae noGUI=main.py`. После правок проверь синтаксис под Python 2.7 (без f-строк, без `print()` с запятыми и т.п.).

## Контекст

2D пластина 150 x 50 мм, толщина 5 мм, два отверстия R = 10 мм в (30; 25) и (120; 25). Две заклёпки (внешний R = 10, внутреннее отверстие R = 4). E = 200000 МПа, nu = 0.3. Контакт без трения, элементы CPS4R. Шаги: Static (`BaseStep`) + Static Linear Perturbation (`LoadStep`). ГУ: u1 = u2 = ur3 = 0 по внутренним отверстиям заклёпок. Нагрузка: 1000 Н/мм2 на всю верхнюю грань AB. Нужно получить напряжение в точке E = (75; 25) мм.

## Правка 1. Убрать партицию пластины

Удали из `create_model()` весь блок целиком (партиция не входит в условие задачи, в расчёте используется ближайший к E узел):

```python
# Partition plate at midpoint x = POINT_E_X to ensure node at point E
log('Partitioning plate at x = ' + str(POINT_E_X) + '...')
dp = plate.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=POINT_E_X)
plate.PartitionFaceByDatumPlane(datumPlane=plate.datums[dp.id],
                                faces=plate.faces[:])
```

Верхняя грань пластины должна остаться одним ребром длиной 150 мм.

## Правка 2. Полная сила с учётом толщины

Заменить:

```python
TOTAL_FORCE = PRESSURE * PLATE_LENGTH
```

на:

```python
TOTAL_FORCE = PRESSURE * PLATE_LENGTH * PLATE_THICKNESS   # 750000 N
```

## Правка 3. Проверка верхней грани

В `create_model()` сразу после `if len(top_edge) == 0: raise ...` добавь:

```python
top_len = sum([e.getSize(printResults=False) for e in top_edge])
log('Top edges: %d, total length: %.1f mm' % (len(top_edge), top_len))
if abs(top_len - PLATE_LENGTH) > 0.5:
    raise RuntimeError('Top edge length %.1f != %.1f' % (top_len, PLATE_LENGTH))
```

## Правка 4. Надёжный `ring_edges`

Замени функцию целиком:

```python
def ring_edges(inst, cx, cy, r):
    """Edges of instance inst lying on circle of radius r centered at (cx, cy)."""
    pts = []
    for e in inst.edges:
        p = e.pointOn[0]
        if abs(hypot(p[0] - cx, p[1] - cy) - r) < 0.5:
            pts.append(p)
    if not pts:
        raise RuntimeError('ring_edges: no edges found for r=%.1f' % r)
    return inst.edges.findAt(*[(p,) for p in pts])
```

## Правка 5. Запросы вывода для всех шагов

Замени блок `model.fieldOutputRequests['F-Output-1'].setValues(...)` на:

```python
for nm in model.fieldOutputRequests.keys():
    model.fieldOutputRequests[nm].setValues(
        variables=('S', 'U', 'RF', 'CSTRESS'))
log('Field output requests: ' + str(model.fieldOutputRequests.keys()))
```

Если `LoadStep` не получил собственного запроса и список не покрывает оба шага, создай явный запрос для него:

```python
if 'F-Output-LoadStep' not in model.fieldOutputRequests.keys():
    model.FieldOutputRequest(name='F-Output-LoadStep', createStepName='LoadStep',
                             variables=('S', 'U', 'RF', 'CSTRESS'))
```

(Вызывай это только если `LoadStep` реально не имеет своего запроса; иначе Abaqus выдаст ошибку дублирования.)

## Правка 6. Резервный треугольный элемент

Замени назначение типов элементов на:

```python
elemType = mesh.ElemType(elemCode=CPS4R, elemLibrary=STANDARD)
triType = mesh.ElemType(elemCode=CPS3, elemLibrary=STANDARD)
plate.setElementType(regions=(plate.faces[:],), elemTypes=(elemType, triType))
rivet.setElementType(regions=(rivet.faces[:],), elemTypes=(elemType, triType))
```

## Правка 7. Только ASCII в файле

- В докстринге `ring_edges` уже ASCII (см. правку 4). Проверь весь файл на кириллицу и символ `Ø`.
- В `extract_results()` строку

  ```python
  f.write('Rivets: Ø%.1f mm with central hole Ø%.1f mm\n\n' % ...)
  ```

  замени на:

  ```python
  f.write('Rivets: D=%.1f mm with central hole D=%.1f mm\n\n' %
          (2*RIVET_OUTER_RADIUS, 2*RIVET_INNER_RADIUS))
  ```
- В комментарии у `RIVET_OUTER_RADIUS` и `HOLE_RADIUS` замени `Ø` на `D=`.

## Правка 8. Контроль реакций и контакта в `extract_results()`

После строки `frame = step.frames[-1]` добавь блок. Он не должен ронять скрипт, если вывода нет:

```python
# --- Check: sum of reactions at fixed rivet holes ---
try:
    rf = frame.fieldOutputs['RF']
    tot = [0.0, 0.0]
    for nm in ('RIVETLEFTINNER', 'RIVETRIGHTINNER'):
        sub = rf.getSubset(region=odb.rootAssembly.nodeSets[nm])
        for v in sub.values:
            tot[0] += v.data[0]
            tot[1] += v.data[1]
    log('Sum RF: RF1=%.1f, RF2=%.1f (expected RF2 ~ +%.0f)' %
        (tot[0], tot[1], TOTAL_FORCE))
except Exception, err:
    log('RF check skipped: ' + str(err))

# --- Check: contact pressure ---
cpress_max = None
try:
    cp_field = frame.fieldOutputs['CPRESS']
    cpress_max = max([v.data for v in cp_field.values])
    log('Max CPRESS = %.3f MPa' % cpress_max)
except Exception, err:
    log('CPRESS not available: ' + str(err))
```

Сохрани `tot` и `cpress_max` в переменных верхнего уровня функции (объяви `tot = [0.0, 0.0]` и `cpress_max = None` до блоков `try`, чтобы использовать ниже).

## Правка 9. Дополнить `summary.txt`

В файле итогов добавь строки:

- в разделе `LOAD`: `Total force (pressure x length x thickness): %.0f N`;
- после результатов в точке E: суммарные реакции `RF1`, `RF2` и `Max CPRESS` (или текст `n/a`, если значение `None`);
- в разделе `MESH` вместо `Total nodes: < 1000` записать фактическое число узлов. Для этого верни его из `create_model()` (например, через глобальную переменную `TOTAL_NODES`) или посчитай заново;
- строку с расстоянием до ближайшего узла оставить, добавить рядом текст: `Value is taken at the nearest node to E (no partition used).`

## Что не менять

- Контактные пары `SurfaceToSurfaceContactStd` (`Contact-L`, `Contact-R`), `FRICTIONLESS`, `sliding=SMALL`.
- `PreLoad` = 1.0 в `BaseStep` и `TopPressure` = `PRESSURE` в `LoadStep` на одной поверхности `PlateTopSurface`.
- Чтение `S` с `position=ELEMENT_NODAL` и усреднение по узлу, индексы `data[0..3]` = S11, S22, S33, S12.
- ГУ на внутренние края заклёпок, `YOUNG_MODULUS = 200000.0`.
- Построение окружностей через `Arc3Points`.

## Критерии готовности

1. `python -m py_compile main.py` (или эквивалентная проверка синтаксиса Python 2.7) проходит.
2. В файле нет не-ASCII символов.
3. В логе: `Top edges: 1, total length: 150.0 mm`, число узлов меньше 1000.
4. В `summary.txt` есть S11, S22, S12, Mises в точке E, расстояние до узла, `RF1/RF2`, `Max CPRESS`.
5. Ожидаемо RF2 около +750000 Н, RF1 около 0. Если знак обратный, в `a.Surface(name='PlateTopSurface', ...)` замени `side1Edges` на `side2Edges`.
