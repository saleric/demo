Для сборки проекта использовалась IDE: CooCox IDE v1.7.8
с компилятором gcc входящим в тулчейн "gcc-arm-none-eabi-4_7-2013q3" от Texas Instruments,
который автоматом подцепился к IDE

Вместо логов компиляции есть листинг сообщений, которые выводились в консоль в процессе сборки.
Он находится в ./Logs/building.log

Итоговые файлы программы (*.bin и *.hex) находятся в директории ./NAMITestJob/Debug/bin

Сам проект создан и собран для чипа STM32F407VG, а именно для отладочной платы STM32F4-Discovery.
Частота тактирования ядра 96МГц.
Шины периферии APB1 и APB2 тактируются сигналом 24МГц.
Все таймеры запущены от 48МГц.

В силу особенностей архитектуры, не удалось сделать структуру битовых полей, 
при записи в которую 1 или 0 изменяло бы значение пина.
Атомарный доступ к пинам портов осуществляется через регистр BSRR соответсвующего порта.
Устройство этого 32-х битного регистра такого, что при записи единицы в младшие 16 разрядов,
на соответствующих пинах порта утанавливается высокий логический уровень. А при записи единицы
в старшие разряды, состояние пина сбрасывается в ноль.
Поэтому структуру атомарного доступа пришлось разбить на поля для установки в "лог. 1" и на 
поля для установки в "лог. 0". Соотвественно вложенные структуры "BitSet" и "BitReset".