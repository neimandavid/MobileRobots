robot = raspbot(); %Setup robot
sendVelocity(robot, 0, 0); %Stop
'Initialized' %Print stuff

while true
    sendVelocity(robot, 0.02, 0.02); %Go
    wait(0.5); %Feed thread and watchdog
end