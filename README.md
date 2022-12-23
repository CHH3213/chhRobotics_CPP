## 自动驾驶决策规划控制c++代码实现

理论知识参考[博客](https://blog.csdn.net/weixin_42301220/article/details/124832403).


python代码实现参考[github仓库](https://github.com/CHH3213/chhRobotics) .

> 推荐结合博客内容来看代码实现。





### 项目依赖
本项目在Ubuntu 20.04下运行，windows下尚未尝试过，因此推荐使用Ubuntu系统。

```markdown
- python3
- matplotlib
- cmake
- Eigen
```


- cmake的安装直接终端运行
    ```shell
    sudo apt install cmake
    ```
    
    如果在项目编译时报cmake版本低的错误，可参考该 [博客](https://www.cnblogs.com/wzc0066/p/16504557.html) 升级cmake。
    
    对cmake操作不不够熟悉的同学可以先参考[文档](https://github.com/CHH3213/Books/blob/master/%E7%BC%96%E7%A8%8B/%E5%B7%A5%E5%85%B7/CMake%20Practice.pdf) 学习。

- Eigen在Linux下的安装直接使用命令

    ```shell
    sudo apt-get install libeigen3-dev
    ```
    
    Eigen库采用模板编程技术，仅由一些头文件组成，运行速度快。用cmake管理项目的时候，只需要在CMakeLists.txt里面添加头文件的路径即可：
    ```cmake
    find_package(Eigen3 REQUIRED)
    include_directories(${EIGEN3_INCLUDE_DIR})
    ```
    
    Eigen库的学习除了[官网](https://eigen.tuxfamily.org/index.php?title=Main_Page#Documentation) 之外，还可以参考[这篇博客](https://blog.csdn.net/hongge_smile/article/details/107296658#t1) 。


- 画图代码采用了c++ 调用python的matplotlib的方式，所以需要使用`pip`的方式安装matplotlib，具体使用方式参考[说明文档](https://matplotlib-cpp.readthedocs.io/en/latest/compiling.html#compiling) 。
这边直接移植了该功能。


- 另外，MPC代码使用了CPPAD/Ipopt优化库，如果要进行MPC的测试，需要安装CPPAD/ipopt。安装方式可参考[博客](https://blog.csdn.net/weixin_42301220/article/details/127946528) 。


### 项目编译

本项目使用Cmake编译，所使用IDE为Clion（推荐使用）。

若不使用Clion，则在项目主目录下，编译：

```shell
mkdir build
cd build
cmake ../
make
```




### 路径规划
- [全局路径规划算法——Dijkstra算法](https://blog.csdn.net/weixin_42301220/article/details/125060298?spm=1001.2014.3001.5501)
- [全局路径规划算法——蚁群算法](https://blog.csdn.net/weixin_42301220/article/details/125129090?spm=1001.2014.3001.5501)
- [全局路径规划算法——动态规划算法](https://blog.csdn.net/weixin_42301220/article/details/125136221?spm=1001.2014.3001.5501)
- [全局路径规划算法——A*算法](https://blog.csdn.net/weixin_42301220/article/details/125140910?spm=1001.2014.3001.5501)
- [局部路径规划算法——曲线插值法](https://blog.csdn.net/weixin_42301220/article/details/125153270)
- [局部路径规划算法——人工势场法](https://blog.csdn.net/weixin_42301220/article/details/125155505)
- [局部路径规划算法——贝塞尔曲线法](https://blog.csdn.net/weixin_42301220/article/details/125167672)
- [局部路径规划算法——B样条曲线法](https://blog.csdn.net/weixin_42301220/article/details/125173884)
- [局部路径规划算法——DWA算法](https://blog.csdn.net/weixin_42301220/article/details/127769819?spm=1001.2014.3001.5502)
- [基于采样的路径规划算法——PRM](https://blog.csdn.net/weixin_42301220/article/details/125254296)
-  [基于采样的路径规划算法——RRT](https://blog.csdn.net/weixin_42301220/article/details/125254061?spm=1001.2014.3001.5501)
- [基于采样的路径规划算法——RRT-Connect](https://blog.csdn.net/weixin_42301220/article/details/125267028?spm=1001.2014.3001.5501)
- [基于采样的路径规划算法——RRT*](https://blog.csdn.net/weixin_42301220/article/details/125275337)
- [路径规划—— Dubins 曲线推导(基于向量的方法)](https://blog.csdn.net/weixin_42301220/article/details/125328823)
- [路径规划—— Dubins 曲线公式总结(基于几何的方法)](https://blog.csdn.net/weixin_42301220/article/details/125493646)
- [路径规划——ReedsShepp 曲线总结](https://blog.csdn.net/weixin_42301220/article/details/125382518)
- [汽车速度规划介绍](https://blog.csdn.net/weixin_42301220/article/details/125831886)


### 决策控制
- [PID实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/124793474)
- [PurePursuit实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/124882144?spm=1001.2014.3001.5501)
- [Stanley实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/124899547)
- [后轮位置反馈实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/125003918?spm=1001.2014.3001.5501)
- [LQR控制算法](https://blog.csdn.net/weixin_42301220/article/details/124542242)
- [LQR控制实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/125031348?spm=1001.2014.3001.5501)
- [模型预测控制(MPC)实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/124566369)

### 预测
- [学习卡尔曼滤波（一）——线性卡尔曼滤波](https://blog.csdn.net/weixin_42301220/article/details/124578094)
- [学习卡尔曼滤波（二）——扩展卡尔曼滤波](https://blog.csdn.net/weixin_42301220/article/details/124605350)
- [学习卡尔曼滤波（三）——无迹卡尔曼滤波](https://blog.csdn.net/weixin_42301220/article/details/124708187)




