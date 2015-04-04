#MSRA Coding Challenge
##Bilateral Filter
-Hang Chu, Cornell University


###Compile & Run
Before compiling this code, make sure you have the following dependencies installed on your computer:

* OpenCV
* PCL
* CMake

Then do the following:

		mkdir build
		cd build
		cmake ..
		make
		./bilateral_filter ../rgb.png ../depth.png 0 10 2.0 20.0
		./bilateral_filter ../rgb.png ../depth.png 1 10 2.0 6.0

###Write-up
Please refer to the file *write-up.txt*. The tex source files are at *./write-up/*.