# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found in the classroom lesson for the EKF project.

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`



# 一、无迹卡尔曼滤波器

> 卡尔曼滤波适用于线性系统，针对于非线性系统很好推广应用。EKF（扩展卡尔曼滤波）利用线性化的方式，让状态和协方差在线性化方程中传播，但是面对强非线性，这种方式误差较大，因为高斯分布的噪声经过非线性系统的分布并不是高斯分布。UKF利用多个采样点（无迹变换）在非线性系统中传播，降低了随机变量经过非线性系统传播的误差，效果强于EKF。
> 与EKF（扩展卡尔曼滤波）不同,UKF是通过无损变换使非线性系统方程适用于线性假设下的标准Kalman滤波体系,而不是像EKF那样,通过线性化非线性函数实现递推滤波。目标跟踪有两个理论基础,即数据关联和卡尔曼滤波技术. 由于在实际的目标跟踪中,跟踪系统的状态模型和量测模型多是非线性的,因此采用非线性滤波的方法.

# 二、CTRV模型
在扩展卡尔曼滤波器（EKF）中，我们假设了速度是恒定的，但是这个在现实中显然是不可能的，现实中物体可能会转弯，可能会走弧线，所以EKF的预测精度相对就会较差，因为在EKF中，我们的状态向量是只确定了[P<sub>x</sub>,P<sub>y</sub>,v<sub>x</sub>,v<sub>y</sub>],没有考虑到转向，于是我们就有了新的模型，就是CTRV（恒定转弯率和速度幅度模型）模型。当然，模型不止一种，还有以下的几种模型：

> 恒定转动率和加速度（CTRA） 
> 恒定转向角和速度（CSAV） 
> 恒定曲率和加速度（CCA） 
> 恒定转弯率和速度幅度模型（CTRV）
## 2.1 状态向量
对于CTRV模型，我们的状态向量为[P<sub>x</sub>,P<sub>y</sub>,v,ψ,ψ`]<sup>T</sup>,每一项的含义如下：

- P<sub>x</sub>:X轴位置
- P<sub>y</sub>：Y轴位置
- v：速度的大小，是一个标量
- ψ：偏航角，转向角
- ψ`：偏航角速度，即转向角速度
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200402233747465.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjczNzQ0Mg==,size_16,color_FFFFFF,t_70)
## 2.2 状态转移方程计算
### 2.2.1 确定部分
我们需要从上一个状态量来预测下一个状态，但是显然这个预测的关系我们没有办法直接从图上得出来，但是不着急，我们一步一步的推。
首先，我们可以根据状态算出其变化率（change rate of state）：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403000723744.png)

所以在经过Δt之后，我们认为新的状态向量为：

![在这里插入图片描述](https://img-blog.csdnimg.cn/2020040300065411.png)

这个看起来计算量很大，但是因为状态向量变化率比较特殊，所以没那么难算，经过变换如下图：

![在这里插入图片描述](https://img-blog.csdnimg.cn/2020040300125280.png)

可以使用在线解积分的网站，例如[这里](https://www.wolframalpha.com/examples/)

最终解出来是这样，这个是在ψ'<sub>k</sub>不为0的时候的解：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403005707860.png)

当ψ'<sub>k</sub>为0的时候，则

Px'的积分为：v<sub>k</sub>cos(ψ<sub>k</sub>)Δt

Py'的积分为：v<sub>k</sub>cos(ψ<sub>k</sub>)Δt

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403005751856.png)
### 2.2.2 噪声
到此为止转移方程中确定的部分已经讨论完了，接下来就需要讨论不确定的部分，即各种噪声。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403003113917.png)

我们用一个二维向量来表示噪声向量：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403003204606.png)

这个向量是由两个相互独立的噪声组成，分别如下：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403003311563.png)

这个主要取决于物体的纵向加速度，所以每一个计算步骤中其实都是改变的。并且服从正态分布。均值是0，方差是σ<sub>a</sub><sup>2</sup>。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403003730900.png)

 这个主要取决于物体的角加速度，也是动态的。并且也服从正态分布。
 
所以我们把噪声加入状态转移方程;

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403004536266.png)

在这里我们可以分别求出a，b，c，d，e。
- **a:**

  ![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403004649570.png)

- **b**:
  
  ![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403004738394.png)
- **c**

  ![在这里插入图片描述](https://img-blog.csdnimg.cn/2020040300482364.png)
- **d**

  ![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403004841775.png)
- **e**

  ![在这里插入图片描述](https://img-blog.csdnimg.cn/2020040300485697.png)

因此有：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403005823905.png)

对于ψ'<sub>k</sub>不为0的时候：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403010039404.png)

ψ'<sub>k</sub>为0的时候：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200403010115923.png)

# 三、无迹卡尔曼滤波器
与EKF（扩展卡尔曼滤波）不同,UKF是通过无损变换使非线性系统方程适用于线性假设下的标准Kalman滤波体系,而不是像EKF那样,通过线性化非线性函数实现递推滤波。

## 3.1 无迹卡尔曼滤波思路
首先看下图，这是k时刻的状态向量和状态协方差：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200404005233939.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjczNzQ0Mg==,size_16,color_FFFFFF,t_70)

其中椭圆表示协方差矩阵符合正态分布，我们要做的就是预测k+1时刻的状态向量和状态协方差。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200404005942741.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjczNzQ0Mg==,size_16,color_FFFFFF,t_70)

如果是经过一个线性的变换，那么我们的结果，也就是k+1时刻的状态向量和协方差矩阵仍然符合正态分布。
但是如果时经过非线性的变换，那么就可能会像下图的黄线区域，不再符合正态分布。而UKF的目的，就是找到一个正态分布，尽量接近的去拟合这个非正态分布，例如图中的绿色椭圆。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200404010509820.png)

那么我们的目标就变成了如何寻找这个拟合正态分布。

## 3.2 sigma点
想要使用非线性变换转换整个协方差分布椭圆非常困难，但是如果我们只转换几个点的话式比较容易的。这就是我们要找的sigma点。
sigma点的主要是通过每个状态向量维度经过一定的公式计算，算出的几个点。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200404223600219.png)

如上图，在选好sigma点后，将sigma点带入非线性变换中，就可以得到变换后的sigma点分布，然后根据sigma点的分布拟合出新的一个高斯分布，那么这个高斯分布就基本认为与我们预测的新的状态向量和协方差矩阵相似，就可以认为式预测值。
PS：如果是线性变换，那么使用sigma点你和出来的其实就是普通的卡尔曼变换。

## 3.3 无迹卡尔曼滤波器的实现（预测）
### 3.3.1 计算sigma点
#### 3.3.1.1计算方法
sigma点的数量取决于状态向量的维度，在这里，我们用的基于CTRV模型的状态向量的维度n<sub>x</sub> = 5，根据经验，我们认为选取n<sub>σ</sub> = 2n<sub>x</sub>+1个sigma点比较合适，所以这里选取的n<sub>σ</sub>为11。
其中，第一个点是状态向量的均值，然后其他的每个维度上都有两个点，从本维度上扩散

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200404231027601.png)

sigma点的计算公式如下图：

![在这里插入图片描述](https://img-blog.csdnimg.cn/2020040423412413.png)

其中需要注意的是，λ是一个设计参数，可以选择sigma点的扩散方向，一般根据经验，
人们认为：
λ = 3 - n<sub>x</sub>
是一个比较合适的选择，同时矩阵的开方本来计算比较困难，但是我们可以直接使用函数计算给出。
其中x<sub>k∣k</sub>永远是第一列，

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200404234235194.png)

为第2列到第n<sub>x</sub> +1 列，

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200404234328173.png)

为第n<sub>x</sub> +2 列到第2n<sub>x</sub> +1 列。

#### 3.3.1.2 关于噪声向量
在卡尔曼滤波器中，说到噪声向量，这个表述有时候不是很明确，有的时候是指下图红框：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405000127882.png)

这个噪声向量跟Δt相关，表示了在两个步骤之间每一个状态向量的维度所受的影响。一般在标准卡尔曼滤波器中是指这个。
而有的时候噪声向量是指这个：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405000439813.png)

这个向量只是列出了独立的噪声，没有说明这些噪声对于过程的影响。 在无迹卡尔曼滤波器中一般指这个。

我们知道噪声协方差矩阵Q的计算公式为：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405000724762.png)

Q的一个对角线上是0 是因为纵向加速度噪声和偏向角加速度噪声是独立的。

#### 3.3.1.3 带噪声的sigma点计算
因此当我们考虑到噪声，我们就需要使用增广矩阵，我们的状态向量就变成了

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405001210931.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjczNzQ0Mg==,size_16,color_FFFFFF,t_70)

所以n<sub>x</sub> = 7，那么n<sub>σ</sub> = 2n<sub>a</sub>+1 = 15。
 这个时候增广协方差矩阵就是：

 ![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405001705940.png)

### 3.3.2 sigma点的预测
对于sigma点的预测，这里比较简单，只需要将其带入到我们之前根据模型得出的非线性变换的公式中，但是值得注意的是，我们输入的状态向量是带有噪声的，但是预测的状态向量是不带噪声的，因此需要做一个维度上的变换。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405002754411.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjczNzQ0Mg==,size_16,color_FFFFFF,t_70)

```cpp
  VectorXd a = VectorXd(n_x_);
  VectorXd u = VectorXd(n_x_);
  VectorXd xk = VectorXd(n_x_);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yaw_dot = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    xk = Xsig_aug.col(i).segment(0, n_x_); // 7 => 5
    u[0] = 0.5 * (delta_t * delta_t) * cos(yaw) * nu_a;
    u[1] = 0.5 * (delta_t * delta_t) * sin(yaw) * nu_a;
    u[2] = delta_t * nu_a;
    u[3] = 0.5 * (delta_t * delta_t) * nu_yawdd;
    u[4] = delta_t * nu_yawdd;

    //avoid division by zero
    if (fabs(yaw_dot) > 0.001)
    {
      a(0) = (v / yaw_dot) * (sin(yaw + yaw_dot * delta_t) - sin(yaw));
      a[1] = (v / yaw_dot) * (cos(yaw) - cos(yaw + yaw_dot * delta_t));
      a[2] = 0;
      a[3] = yaw_dot * delta_t;
      a[4] = 0;
    }
    else
    {
      a[0] = v * cos(yaw) * delta_t;
      a[1] = v * sin(yaw) * delta_t;
      a[2] = 0;
      a[3] = yaw_dot * delta_t;
      a[4] = 0;
    }
    //write predicted sigma points into right column
    Xsig_pred_.col(i) = xk + a + u;
  }
```
### 3.3.3 均值和方差的预测
我们使用权重来根据sigma点计算预测的均值和协方差矩阵，具体公式如下，其中i是每一列：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405135848655.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjczNzQ0Mg==,size_16,color_FFFFFF,t_70)

因此，我们就需要计算对于每一列的权重：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405150003262.png)

我们看到权重的计算主要是跟λ相关，λ是一个设计参数，可以选择sigma点的扩散方向，所以我们在计算权重的时候也是考虑的这个参数。
因此，X和P的计算公式就是：

![在这里插入图片描述](https://img-blog.csdnimg.cn/202004051500359.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjczNzQ0Mg==,size_16,color_FFFFFF,t_70)


```cpp
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++)
  {
    weights_(i) = 1 / (2 * (lambda_ + n_aug_));
  }
```

```cpp
  //mean
   x_.fill(0.);   // important 
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  //Covariance
  P_.fill(0.); //important
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    /*
	这里涉及到了角度，所以需要将角度标准化
    */
    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
```
## 3.4 无迹卡尔曼滤波器的实现（更新）
### 3.4.1 预测测量结果
 到目前为止，我们已经成功的使用了k时刻的状态预测了k+1时刻的状态，我们需要将状态向量转换到测量向量空间（即将状态向量转化成与测量向量相同的结构）。
这个过程仍然是一个非线性的变换，所以是以下的一个过程：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405142748552.png)

因为是一个非线性的变换，其实我们在这里还可以使用相同的方式，即选取sigma点，然后变换，然后拟合。但是这里我们可以偷一下懒，我们可以直接使用现成的我们之前计算过的sigma点，所以可以少计算一步。
所以整体的计算步骤和公式如下图：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405143243381.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjczNzQ0Mg==,size_16,color_FFFFFF,t_70)

#### 3.4.1.1 雷达数据的处理
对于雷达来说，我们测量到的数据是一个三维的空间，所以我们要将sigma进行一个转换，如下图：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405143121274.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjczNzQ0Mg==,size_16,color_FFFFFF,t_70)

需要用到的公式有：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405152555932.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405152613467.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405152634368.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405152643624.png)

```cpp
//set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  // measurement covariance noise matrix R
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  //calculate Zsig
  Zsig.fill(0.);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yawd = Xsig_pred_(3,i);

    double ro = sqrt(px * px + py * py);
    double theta = atan2(py, px);
    double ro_dot = (px * cos(yawd) * v + py * (sin(yawd) * v)) / ro;


    Zsig(0,i) = ro;
    Zsig(1,i) = theta;
    Zsig(2,i) = ro_dot;
  }

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }
  
  std::cout << "z_pred R:" << z_pred << std::endl;
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;
    S += weights_(i) * (z_diff) * (z_diff).transpose();
  }
  S = S + R;
```
#### 3.4.1.2 激光雷达数据的处理

对于激光雷达，我们测量到的数据是一个二维的空间，所以有：

```cpp
  //set measurement dimension,px,py
  int n_z = 2;
  // measurement covariance noise matrix R
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0.);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  //calculate Zsig
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // VectorXd x = Xsig_pred_.col(i);
    // double px = x(0);
    // double py = x(1);

    // Zsig.col(i)(0) = px;
    // Zsig.col(i)(1) = py;
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  //calculate mean predicted measurement
  z_pred.fill(0.); // important
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  //calculate measurement covariance matrix S
  S.fill(0.); // important
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * (z_diff) * (z_diff).transpose();
  }
  S = S + R;
```

### 3.4.2 UKF 更新
在这一步，我们终于要引入测量值，就是比我们实际上从传感器上拉取回来的值，这里需要 计算的是卡尔曼增益K，在计算过程中，我们需要用到一个相关矩阵，用来表示状态空间中的sigma点和测量空间中的sigma点的关系。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405155527647.png)

所以**卡尔曼增益K**就是

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405155723683.png)

状态向量和协方差矩阵的更新为：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405160205485.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200405160222201.png)

#### 3.4.2.1 雷达数据更新

```cpp
 //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // T
  Tc.fill(0.); //important
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd t_x_diff = (Xsig_pred_.col(i) - x_);
    while (t_x_diff(3) > M_PI)
      t_x_diff(3) -= 2. * M_PI;
    while (t_x_diff(3) < -M_PI)
      t_x_diff(3) += 2. * M_PI;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;
    Tc += weights_(i) * t_x_diff * ((z_diff).transpose());
  }
  // std::cout << "Tc R:" << Tc << std::endl;
  MatrixXd K = Tc * (S.inverse());
  // std::cout << "K R:" << K << std::endl;

  //measurement of radar data
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_(0),
      meas_package.raw_measurements_(1),
      meas_package.raw_measurements_(2);
  // std::cout << "Z measurement:" << z << std::endl;

  VectorXd z_diff = z - z_pred;

  while (z_diff(1) > M_PI)
    z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI)
    z_diff(1) += 2. * M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
```

#### 3.4.2.2 激光雷达数据更新

```cpp
 //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0);  // important
  // T
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd t_x_diff = (Xsig_pred_.col(i) - x_);
    VectorXd z_diff = Zsig.col(i) - z_pred;
    Tc += weights_(i) * t_x_diff * ((z_diff).transpose());
  }
  MatrixXd K = Tc * (S.inverse());
  //measurement of lidar data
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_(0),
      meas_package.raw_measurements_(1);

  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
```
# 四、关于噪声
对于我们的CTRV模型，我们的噪声有两个：
- σ<sub>a</sub><sup>2</sup>,代表了纵向加速度噪声，也可以理解为线性加速度噪声。
- σ<sub>ψ''</sub><sup>2</sup>，代表了角加速度噪声。

在项目中，这两个值往往是来源于硬件的参数，也需要进行一些调整，通常需要一些预估，可以使用被追踪物体的最大加速度的一半作为参考值，然后进行微调。

通常要重复这个过程多次：
- 猜一个差不多的参考值
- 运行UKF
- 观察结果是否满意
- 调整参数重试

在这里我们使用卡方分布查表来获取相应的比较合适的噪声值。




