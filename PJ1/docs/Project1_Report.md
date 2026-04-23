# 计算机图形学 Project 1 实验报告

#### 一、实验信息

- Project 1 曲线和曲面造型技术
- 组员：洪运 23300240019 / 肖佳洲 23300240017


#### 三、代码实现说明

#### 1. 任务1：曲线绘制与局部坐标系

##### 1.1 分段三次 Bezier 曲线 `evalBezier`

实现思路：

1. 校验输入控制点个数满足 `3n+1`。
2. 将控制点按每 4 个一组（相邻段共享端点）遍历。
3. 每一段调用 `evalBezierPiece` 做离散采样。
4. 采样时计算位置与一阶导：
   - 位置：
     \[
     \mathbf{V}(t)=(1-t)^3\mathbf{P}_0+3(1-t)^2t\mathbf{P}_1+3(1-t)t^2\mathbf{P}_2+t^3\mathbf{P}_3
     \]
   - 切向（未归一化）：
     \[
     \mathbf{V}'(t)=3(1-t)^2(\mathbf{P}_1-\mathbf{P}_0)+6(1-t)t(\mathbf{P}_2-\mathbf{P}_1)+3t^2(\mathbf{P}_3-\mathbf{P}_2)
     \]
5. 采样完成后统一调用 `buildCurveFrame` 构造 T/N/B，保证正交并归一化。

细节处理：

- 段与段拼接时，后续段跳过第一个采样点，避免重复顶点。
- 对零长度切向量做回退（优先沿用前一个切向）。
- 若检测到整条曲线在 xy 平面内（`z≈0`），使用固定 `B=(0,0,1)`，再用 `N = B x T`，能明显提升稳定性。
- 非平面曲线用“逐点传播”方式更新法向和次法向，避免局部突变。

##### 1.2 B 样条曲线 `evalBspline`

实现采用“基变换到 Bezier 再复用”策略：

- 对每个 B 样条片段 `P0,P1,P2,P3`，先转换为等价三次 Bezier 控制点：
  \[
  \mathbf{B}_0=\frac{\mathbf{P}_0+4\mathbf{P}_1+\mathbf{P}_2}{6},\;
  \mathbf{B}_1=\frac{4\mathbf{P}_1+2\mathbf{P}_2}{6}
  \]
  \[
  \mathbf{B}_2=\frac{2\mathbf{P}_1+4\mathbf{P}_2}{6},\;
  \mathbf{B}_3=\frac{\mathbf{P}_1+4\mathbf{P}_2+\mathbf{P}_3}{6}
  \]
- 拼接得到整条 Bezier 控制点序列后，直接调用 `evalBezier`。

这样可复用一套采样与 TNB 计算代码，减少重复实现。

##### 1.3 颜色与可视化

当前程序对曲线及局部坐标系的绘制符合要求：

- 曲线：白色
- N：红色
- B：绿色
- T：蓝色

对应实现见 `recordCurve` 与 `recordCurveFrames`。

#### 2. 任务2：曲面绘制

对应文件：`PJ1/src/surf.cpp`

##### 2.1 旋转曲面 `makeSurfRev`

核心流程：

1. 检查 profile 曲线必须在 xy 平面。
2. 沿 y 轴做 `steps+1` 次角度采样（含首尾闭合环）。
3. 对 profile 上每个点做 `rotateY(theta)` 得到顶点位置。
4. 法线采用 profile 法线旋转后取反（`-profile[i].N`），并归一化。
5. 用统一函数 `appendTriangleStripFaces` 按相邻环连接成三角形网格。

细节说明：

- 使用 `steps+1` 环可保证 `theta=0` 与 `theta=2pi` 闭合。
- 面片索引采用一致的顶点绕序，以配合背面剔除。

##### 2.2 广义圆柱面 `makeGenCyl`

核心流程：

1. profile 必须是 xy 平面曲线；sweep 提供空间轨迹与局部坐标系。
2. 先拷贝 sweep 的 `(N,B,T,V)` 作为每个截面局部坐标系。
3. 对每个 sweep 点，把 profile 点映射到世界坐标：
   \[
   \mathbf{p}_{world}=\mathbf{V}+x\mathbf{N}+y\mathbf{B}+z\mathbf{T}
   \]
4. profile 法线同样做取反后映射到世界坐标并归一化。
5. 使用与旋转曲面相同的三角形连接函数生成网格。

#### 3. 闭合问题修正（拓展任务）

位置：`makeGenCyl` 内部 `closedSweep` 分支。

检测条件：

- sweep 起点和终点位置近似相等；
- sweep 起点和终点切向夹角很小（点积大于阈值）。

修正方法：

1. 计算起止法向之间绕切向轴的角差 `alpha`。
2. 对每个截面按参数 `t=j/(ringCount-1)` 线性插值旋转角 `theta=alpha*t`。
3. 将该旋转逐步施加到每个截面的法向 `N`，再用 `B = T x N` 重建次法向。

效果：

- 能修复闭合扫掠中“首尾位置对齐但法向不对齐”的接缝扭转问题；
- 对非闭合曲线不触发该分支，不影响普通样例。

#### 四、结果展示（截图）


#### 1. circles.swp

![](circles1.png)
![](circles2.png)

#### 2. core.swp

![](core1.png)
![](core2.png)

#### 3. flircle.swp

![](flircle1.png)
![](flircle2.png)
![](flircle3.png)

#### 4. florus.swp

![](florus1.png)
![](florus2.png)
![](florus3.png)

#### 5. gentorus.swp

![](gentorus1.png)
![](gentorus2.png)
![](gentorus3.png)

#### 6. norm.swp

![](norm1.png)
![](norm2.png)
![](norm3.png)

#### 7. tor.swp

![](tor1.png)
![](tor2.png)
![](tor3.png)

#### 8. weird.swp（文件名对应截图前缀为 wierd）

![](wierd1.png)
![](wierd2.png)
![](wierd3.png)

#### 9. weirder.swp

![](wierder1.png)
![](wierder2.png)
![](wierder3.png)

#### 10. wineglass.swp

![](wineglass1.png)
![](wineglass2.png)
![](wineglass3.png)

#### 五、总结

- 曲线部分：已实现 Bezier 与 B-spline 的离散采样、切向计算、TNB 构建与可视化。
- 曲面部分：已实现旋转曲面和广义圆柱面，法线方向为外法线。
- 闭合修正：已实现基于旋转差插值的闭合扭转修正，对 `weirder.swp` 生效。

本次实验的核心是把“曲线离散化 + 局部坐标系传播 + 曲面扫掠”完整串起来。实现时最关键的不是公式本身，而是数值稳定性和坐标系一致性