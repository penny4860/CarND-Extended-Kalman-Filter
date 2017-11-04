all : clean EKF

EKF : 
	@echo building EKF.exe: $@
	@mkdir debug
	@g++ -O0 -g3 -Wall -c -fmessage-length=0 -o "debug\\kalman_filter.o" "src\\kalman_filter.cpp"
	@g++ -O0 -g3 -Wall -c -fmessage-length=0 -o "debug\\tools.o" "src\\tools.cpp"
	@g++ -O0 -g3 -Wall -c -fmessage-length=0 -o "debug\\main.o" "src\\main.cpp"
	@g++ -O0 -g3 -Wall -c -fmessage-length=0 -o "debug\\FusionEKF.o" "src\\FusionEKF.cpp"
	@g++ -o EKF.exe "debug\\FusionEKF.o" "debug\\kalman_filter.o" "debug\\main.o" "debug\\tools.o"
	@rm -rf debug
	@echo ''

clean :
	@echo ''
	@echo CLEAN Binary 
	@rm -rf debug
	@echo ... remove obj, dependency files, execute files
	@echo ''
