### Описание пайплайна
ROS пакет planner содержит узлы mapplanner.py, Agent1.py, Agent2.py, Agent3.py.
В планировании используются библиотеки mapspatial, mapmulti, mapcore, robot-act. Их 
адаптация внутри папки scripts
### Запуск
1. Создаем ROS-окружение
2. Создаем ROS-пакет
3. Копируем папку scripts
4. chmod +x на Agent* и mapplanner.py
5. catkin_make на уровне ROS-окружения
####В отдельных терминалах:
1. source devel/setup.bash
2. rosrun planner mapplanner.py

rosrun planner Agent1.py

rosrun planner Agent2.py


### Dependencies
ROS melodic/kinetic
