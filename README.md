# image segmentation
c++ Implementation of Parameter-free Hierarchical Image Segmentation  
[pdf download](Parameter-free%20Hierarchical%20Image%20Segmentation.pdf)  
Algorithm is graph-based mainly on MST and union find.  
CIEDE2000: lab color space distance measurement, got from github  

### test1: without post processing
You can see that without post processing, there are many small components
along edges.
#### rgb average result
![img](segmentation/test/test1/test_rgb_ave/level1.png)
![img](segmentation/test/test1/test_rgb_ave/level2.png)
![img](segmentation/test/test1/test_rgb_ave/level3.png)
![img](segmentation/test/test1/test_rgb_ave/level4.png)
![img](segmentation/test/test1/test_rgb_ave/level5.png)
![img](segmentation/test/test1/test_rgb_ave/level6.png)
![img](segmentation/test/test1/test_rgb_ave/level7.png)
![img](segmentation/test/test1/test_rgb_ave/level8.png)
#### lab average result
![img](segmentation/test/test1/test_lab_ave/level1.png)
![img](segmentation/test/test1/test_lab_ave/level2.png)
![img](segmentation/test/test1/test_lab_ave/level3.png)
![img](segmentation/test/test1/test_lab_ave/level4.png)
![img](segmentation/test/test1/test_lab_ave/level5.png)
![img](segmentation/test/test1/test_lab_ave/level6.png)
![img](segmentation/test/test1/test_lab_ave/level7.png)
![img](segmentation/test/test1/test_lab_ave/level8.png)
### test2: with post processing
With post processing, look much cleaner
#### rgb average result
![img](segmentation/test/test2/test_rgb_ave/level1.png)
![img](segmentation/test/test2/test_rgb_ave/level2.png)
![img](segmentation/test/test2/test_rgb_ave/level3.png)
![img](segmentation/test/test2/test_rgb_ave/level4.png)
![img](segmentation/test/test2/test_rgb_ave/level5.png)
![img](segmentation/test/test2/test_rgb_ave/level6.png)
![img](segmentation/test/test2/test_rgb_ave/level7.png)
![img](segmentation/test/test2/test_rgb_ave/level8.png)
#### lab average result
![img](segmentation/test/test2/test_lab_ave/level1.png)
![img](segmentation/test/test2/test_lab_ave/level2.png)
![img](segmentation/test/test2/test_lab_ave/level3.png)
![img](segmentation/test/test2/test_lab_ave/level4.png)
![img](segmentation/test/test2/test_lab_ave/level5.png)
![img](segmentation/test/test2/test_lab_ave/level6.png)
![img](segmentation/test/test2/test_lab_ave/level7.png)
![img](segmentation/test/test2/test_lab_ave/level8.png)