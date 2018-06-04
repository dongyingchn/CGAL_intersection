# CGAL_intersection

1、读取ShapeNet模型，结果如下图

![image](https://github.com/dongyingchn/CGAL_intersection/blob/master/Figures/Initial_model.png)

2、为了与我们的Morphable Model进行坐标系的统一，对ShapeNet模型进行旋转和坐标系的移动，结果如下图

![image](https://github.com/dongyingchn/CGAL_intersection/blob/master/Figures/transformed_model.png)

3、读取tmplt文件（保存的是图像坐标系下线框坐标信息），将坐标系统一至模型坐标系，并且将线框缩放至ShapeNet尺寸（此时，ShapeNet模型是原始尺寸，尺寸较小，计算会快一些），然后选取线框中某些点对ShapeNet模型进行投影，选取的12个点如下图

![image](https://github.com/dongyingchn/CGAL_intersection/blob/master/Figures/points_selected.png)
