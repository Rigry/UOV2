Программа для удаленного и местного управления УЗ, УФ с помощью "сухих" контактов и индикации аварий.<br>

Входа EN_UZ(УЗ), EN_UV(УФ) (указано на плате) для управления выходами UZ_ON, UZ_WORK(УЗ), UV_ON, UV_WORK(УФ) соответственно.<br>
Входа EN_UZ(УЗ), EN_UV(УФ) активны только в местном режиме управления.<br>
Входа UZ_WORK, UV_WORK выполняют роль сигнальных для визуального контроля (например, сигнальных ламп на шкафу управления установкой).<br>
Входа UZ, EPRA для контроля состояния узг и эпры соответственно.<br>
Вход RC для переключения между удаленным и местным режимами. Замкнут - управление удаленное, разомкнут - местное.<br>
Вход RC_ON - для запуска узг и уф одним удаленным сигналом. Оба сигнала RC и RC_ON подобие фиксируемой кнопки. <br>

Выход UV_ALARM - сигнал аварий уф (неисправная лампа/эпра, низкий уровень УФ).<br>
Выход UZ_ALARM - сигнал аварии узг.<br>
Выход OVERHEAT - перегрев установки (датчик температуры заводится на плату). Замкнут - перегрев.<br>

Алгоритм работы:<br>
    при включении уф и отсутствия перегрева установки уровень УФ и состояние эпры не контролируются в течении 2 и 0,5 минуты соответственно (период розжига) лампы. После этого времени сигнал аварий сигнализирует о неисправности установки. Критический уровень УФ - 40%.<br>
    При включении уз и отсутствия перегрева контроль за состоянием узг производится без задержек.<br>
    Контроль за температурным режимом производится при подаче питания на контроллер. При этом управление уф и уз производится автоматически. В случае перегрева контроллер отключит (либо выставит запрет на включение) эпру и узг. Включит (либо выставит разрешение на включение) при возвращении температуры установки в рабочий диапазон (20-40 С).<br>

Дополнение №1 (24.01.2024) Сброс максимума:<br>
    для сброса максимума уровня ультрафиолета (например, при смене отработанной лампы) необходимо:<br>
    1) полностью выключить установку<br>
    2) перевести установку в ручной режим (режим местного управления)<br>
    3) включить установку<br>
    4) после подачи питания в течении 5 сек. три раза вкл и выкл ультрафиолет<br>
Примечание: для контроля сброса максимума логичнее производить сброс на отработанной лампе.<br>
В таком случае ошибка по уровню должна уйти, что свидетельствует об успешном сбросе.<br> 
После установки новой лампы наберется новый максимум.<br>

Дополнение №2 (24.01.2024) Задержка аварии УЗ на 2 сек. после включения.<br>