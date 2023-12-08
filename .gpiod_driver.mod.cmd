cmd_/home/pi/final_project/gpiod_driver.mod := printf '%s\n'   gpiod_driver.o | awk '!x[$$0]++ { print("/home/pi/final_project/"$$0) }' > /home/pi/final_project/gpiod_driver.mod
