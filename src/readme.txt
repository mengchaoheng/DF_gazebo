1，catia：建立好坐标，转换两个文件，一个全部转stp得到plls_all，另一个去掉操纵面转stp得到plls。
2，solidworks：分别打开两个stp，都勾选“不要转换stl输出数据正的坐标空间”。plls_all不勾选“在单一文件中保存装配图的作用零部件”转换，
得到所有零件的stl文件。plls勾选“在单一文件中保存装配图的作用零部件”得到base_link。

注：此时每个零件相对位置为0即组装成功，坐标系使用的是装配图的默认坐标系
---------------------------------------------------------------------------------------
编辑.sdf文件时，注意重心，转动惯量，关节位置。
如模型加载错误（固定在原点），则检查阻尼系数(<damping>0.01</damping>)、控制通道（包括执行器环的pid参数、控制输入正负等，可先注释掉，以排查）、
关节转轴（正方向定义正攻角）、upward/forward方向、舵和桨的力/力矩系数。


--------------------
只改df2和df4两个包，baylands.work, liftdrag_plugin.cpp,liftdrag_plugin.h
================================


Sdf文件要点：
1. link质量属性。
2. joint阻尼以及引擎参数。implicit_spring_damper必须为真，大阻尼（>0.005）时才不会回原点。
3. plugin的joint_control_type.设置为position_kinematic时必须是小质量物体（如px4的模型）。设为position时需要加上适当的p控制参数。

4. 碰撞，按照物理意义定义会散架，可设计box。
主要有两种设置：
======1.px4=====
------link小质量设置-------
      <inertial>
        <mass>0.00000001</mass>
        <inertia>
          <ixx>0.000001</ixx>
          <ixy>0.0</ixy>
          <iyy>0.000001</iyy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
          <izz>0.000001</izz>
        </inertia>
      </inertial>

------joint阻尼及引擎参数-------
      <axis>
        ...
        <dynamics>
          <damping>0.1</damping>
        </dynamics>
      </axis>
      <physics>
        <ode>
          <implicit_spring_damper>1</implicit_spring_damper>
        </ode>
      </physics>

-------plugin控制类型--------
          <joint_control_type>position_kinematic</joint_control_type>
-----------------
======2.自定义=====
------link按照实际物理属性进行设置-------
      <inertial>
        <mass>0.002</mass>
        <inertia>
          <ixx>8.123572e-07</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1.111369e-07</iyy>
          <iyz>0</iyz>
          <izz>7.222304e-07</izz>
        </inertia>
      </inertial>

------joint阻尼及引擎参数-------
      <axis>
        ...
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
          <damping>0.01</damping>
        </dynamics>
      </axis>
      <physics>
        <ode>
          <implicit_spring_damper>1</implicit_spring_damper>
        </ode>
      </physics>

-------plugin控制类型，一定需要position且设置pid参数--------
          <joint_control_type>position</joint_control_type>
          <joint_control_pid>
            <p>1</p>
            <i>0</i>
            <d>0</d>
            <iMax>0</iMax>
            <iMin>0</iMin>
            <cmdMax>20</cmdMax>
            <cmdMin>-20</cmdMin>
          </joint_control_pid>
-----------------
