all : clean EKF

EKF : 
	@echo Building target: $@
	@mkdir src
	@g++ -O0 -g3 -Wall -c -fmessage-length=0 -o "src\\kalman_filter.o" "..\\src\\kalman_filter.cpp"
	@g++ -O0 -g3 -Wall -c -fmessage-length=0 -o "src\\tools.o" "..\\src\\tools.cpp"
	@g++ -O0 -g3 -Wall -c -fmessage-length=0 -o "src\\main.o" "..\\src\\main.cpp"
	@g++ -O0 -g3 -Wall -c -fmessage-length=0 -o "src\\FusionEKF.o" "..\\src\\FusionEKF.cpp"
	@g++ -o EKF.exe "src\\FusionEKF.o" "src\\kalman_filter.o" "src\\main.o" "src\\tools.o"
	@rm -rf src
	@echo ''

clean :
	@echo ''
	@echo CLEAN Binary 
	@rm -rf src
	@echo ... remove obj, dependency files, execute files
	@echo ''
