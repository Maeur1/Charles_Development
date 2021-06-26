L6470: L6470.c examples/robot/robot.c
	gcc -o output/L6470 "examples/robot/robot.c" L6470.c -I. -lm -lwiringPi -lpthread
MUSIC: L6470.c examples/music/music.c
	gcc -o output/MUSIC "examples/music/music.c" L6470.c -I. -lm -lwiringPi -lpthread
