## ДЗ 2:

-   Сделано:
    -   ввод из файла
    -   класс Fixed
    -   Шаблонный класс симулятора с выбором типов
    -   выбор типов и размеров в рантайме
-   В работе:
    -   сохранение в файл на произвольном этапе
-   Не сделано:

## ДЗ 3:

-   Сделано:
    -   оптимизация без параллельности
-   В работе:
    -   тредпул для оптимизации через параллельность
-   Не сделано:
    -   все остальное

## Отчет по оптимизации

-   Список примененных оптимизаций:
    -   Избавление от избыточного ranges::find при выборе из deltas
    -   Добавление атрибута inline где он уместен
    -   Генерация случайных чисел в отдельном потоке и их получение в основном через потокобезопасную очередь (генерация случайного числа определенно медленней обращения к мьютексу)
    -   Применение оптимизации O3

Замеры:

| Условие                                  | Время на 200 тиков |
| ---------------------------------------- | :----------------: |
| До применения оптимизаций                |     ~14 секунд     |
| После применения собственных оптимизаций |    ~5.5 секунд     |
| После применения оптимизации O3          |     ~4 секунды     |
