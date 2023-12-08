cmd_/home/pi/final_project/Module.symvers :=  sed 's/ko$$/o/'  /home/pi/final_project/modules.order | scripts/mod/modpost -m -a    -o /home/pi/final_project/Module.symvers -e -i Module.symvers -T - 
