# Related Work 细查(T5)——检索报告与新颖性边界

日期:2026-08-26。方法:六条并行检索线(①行走机力再分配经典带 ②吸盘阵营
+日文 ③多肢攀爬内力+admittance 邻域 ④干粘附剥离 ⑤横切:弹性释能现象+
前馈补偿方法学 ⑥中文扫雷),合计 ~350 次检索/抓取动作;工具=WebSearch+
arXiv+Semantic Scholar/OpenAlex/Crossref API+J-STAGE+Google Patents。
已知限制:S2 搜索端点大面积 429(OpenAlex/Crossref 补位);IEEE/MDPI/PMC
多 403,主体为摘要级核实,少量正文级(LORIS 全文 PDF、科学通报 2023 综述
全文、机械工程学报 2023 综述全文、《机器人》6 篇 PDF、JSME 関東支部 19A15
全文等);CNKI/万方/维普全文墙内未进。**引用前须人工核全文的条目见 §7。**

---

## 0. 结论速查(逐贡献新颖性边界)

**总判定:无直接撞车。** 没有任何已核实工作同时满足"吸附型爬壁+脱附瞬间
滑移量化+开环无传感器脱附前卸载"中的两项以上——最严格读法下 Stickybot
专利占"吸附爬壁+脱附前卸载"两项,但它是力传感器闭环+干粘附+无滑移量化,
属于必须正面划界的思想同源近邻,不是撞车。

- **贡献①(现象+量化)——安全,可主张首次**(带"据我们所知"):首次对
  "负压密封破裂瞬间弹性储能释放→整机棘轮式位置损失"做时间分辨量化
  (83–93% 集中于破裂后 <100ms;逐周期、逐腿、mm 级)。三个限定词锁死:
  **负压密封破裂瞬间 / 整机位置损失(mm、逐周期)/(解法)开环无传感器**。
  先例全是定性或异口径:Stickybot T-RO 2008 定性记录"脱附力瞬态在机体
  传播、致其余足过早脱附";科学通报 2023 综述(戴组)点名"瞬态力和较大的
  脱附力传递到躯干产生机体震颤"并把"减小脱附冲击"列为待攻关方向;干粘附
  阵营量化的是足端力(N)与机体振动(m/s²),吸盘阵营量化的是腔压
  (潘雷 2005),无人量化整机沿墙位移。
- **贡献②(模型)——安全**:内力概念谱系须引 Kumar & Waldron 1988(闭链
  内力分解)与 DLR Crawler("位置控制的多足机会积累内力"命题);但"交接期
  弹性内力的位移域并联弹簧模型+刚度加权平衡+多条被验证预言"无先例。
- **贡献③(解法)——措辞必须收窄**:❌ 不可写"首次提出脱附前卸载"——
  Stickybot 专利 US7762362B2 原文明确"lift-off 前沿切向卸载、松弛累积的力
  与弹性变形、防瞬态力扰动其他足"(力传感器+刚度控制器闭环);LORIS
  (ICRA 2024)步态状态机有显式 Unload 步(在线力优化把待放爪接触力约束
  到零)。✅ 可主张:**首次把脱附前卸载条件从力域换算到位移域(逐腿离线
  标定 δ 归还+六腿指令均值不变分摊),在无任何力/触觉/电流反馈的位置舵机
  平台上纯开环实现,并以破裂段滑移量化(−82%/−88%)闭环验证**。"离线
  标定表+纯运动学预位移把多接触内力归零"六线均未找到先例。
- **量纲巧合警示**:FTFOF(arXiv 2504.19448)报"振动幅值 −82%"(m/s²),
  与我们 B 组破裂段 −82%(mm)数字巧合、量纲不同,行文必须点破。

**最近邻 Top-6(写作时逐条正面划界)**:Stickybot 专利+T-RO 2008、
LORIS 2024、Ota 2006(吸附平台上的变形补偿唯一先例,但补静态重力变形)、
TITAN XI 2005(位移域标定前馈同手段,目标是落足精度)、Chen 1999("免力
传感器"仅指设定值计算)、Wang21/Wen25/FTFOF 干粘附脱附簇。

---

## 1. A 群:攀爬机器人脱附管理(现象与解法的直接近邻)

- 🔴 Cutkosky & Kim(申 2007/授 2010)"Climbing with dry adhesives" — 美国专利 US7762362B2 — https://patents.google.com/patent/US7762362B2/en — 原文明确"抬脚前须沿切向卸载以松弛累积的力与弹性变形、防止瞬态力",控制律 x_cmd=x_ff+(kP+kI/s)·C·(f_s−f_d) — 思想同源的最早正式陈述;划界=Hall 力传感器闭环 vs 零传感器开环、干粘附 vs 真空密封破裂、无滑移量化 vs 滑移为主指标。
- 🔴 Kim, Spenko, Trujillo, Heyneman, Santos & Cutkosky (2008) "Smooth Vertical Surface Climbing With Directional Adhesion" — IEEE T-RO 24(1):65–74 — DOI 10.1109/TRO.2007.909786 — Stickybot 定向粘附+切向力控玻璃爬行,记录各向同性垫时代"大脱附力瞬态在机体传播、致其余足过早脱附/打滑" — 现象定性先例+材料级解法(近零力脱附靠方向性粘附);无位置损失量化。**原文已核(09-01,MIT 作者公开版存 paper/references/kim2008smooth.pdf)**:逐字引语 "This large detachment force was the main limitation of the isotropic patches, producing oscillations that frequently caused the other feet to slip";另有 "The unloading step for the anisotropic patches is accomplished rapidly and results in negligible detachment force"——paper/main.tex Related Work 已改按原句引用(此前按检索摘要的转述套引号=误引风险,已消)。姊妹篇:Kim et al. ICRA 2007 "Whole body adhesion"(最佳学生论文,力控源头)、Santos et al. ICRA 2007(定向粘附结构;"各向同性垫需大 pull-off 力"引语出自检索摘要,原文未逐字核 (?))。
- 🔴 Nadan, Backus & Johnson (2024) "LORIS: A Lightweight Free-Climbing Robot for Extreme Terrain Exploration" — ICRA 2024 — https://www.ri.cmu.edu/app/uploads/2024/04/LORIS__A_Lightweight_Free_Climbing_Robot_for_Extreme_Terrain_Exploration.pdf(全文已核) — 微刺被动爪四足,步态状态机显式 Unload 步:"constrain the contact force on the gripper to be zero in the force optimization and wait for the contact force to converge" — 与零力交接目标状态相同;划界=本体感知+在线力优化闭环 vs 离线 δ 表开环、微刺岩面 vs 真空吸盘、无滑移量化。
- 🔴 Boscariol, Henrey, Li & Menon (2013) "Optimal gait for bioinspired climbing robots using dry adhesion: A quasi-static investigation" — J. Bionic Engineering 10(1) — DOI 10.1016/S1672-6529(13)60193-6 — 指出预压/脱附一脚引起全机接触力再分配、不当分配致整机脱落,准静态模型离线优化步态 — 占"吸附爬壁+脱附前载荷管理"两要素;划界=刚体接触力规划(仿真层)vs 腿链弹性内力位移域操纵(实机),无 δ 标定、无滑移量化。
- 🟠 Wang, Xiong, Duan, Wang & Dai (2021) "Compliant Detachment of Wall-Climbing Robot Unaffected by Adhesion State" — Applied Sciences 11(13):5860 — DOI 10.3390/app11135860 — 在线阻抗调剥离角至 π,"基本消除脱附末态法向粘附力的瞬间突变" — 同"脱附瞬间突变"问题意识;干粘附、闭环阻抗、足端力口径。
- 🟠 Wen, Zheng, Jing, Guo & Chen (2025) "Force–Position Coordinated Compliance Control in the Adhesion/Detachment Process of Space Climbing Robot" — Aerospace 12(1):20 — DOI 10.3390/aerospace12010020 — 足端力实时反馈+旋转剥离,拉脱力 −65.5%,"减小对本体与支撑腿扰动" — 问题表述最接近的一篇;力反馈闭环、微重力悬吊、无本体滑移量化。
- 🟠 Xiao, Nie, Hao & Li (2025) "An End-to-End Framework for Optimizing Foot Trajectory and Force in Dry Adhesion Legged Wall-Climbing Robots"(FTFOF) — arXiv:2504.19448 — 三段 C² Bezier 足端轨迹+GRU 力预测多目标优化,最大脱附力 7.49→5.40N(−28%)、IMU 振动 327→58 m/s²(−82%) — 只整形被脱附足自身轨迹、无支撑组内力再分配、依赖力测量训练;开/闭环执行细节未核(§7);其引文 21 条无脱附滑移量化条目,唯一施引不相关。⚠ −82% 量纲巧合。
- 🟠 Imai, Uno & Yoshida (2024) "Admittance Control-based Floating Base Reaction Mitigation for Limbed Climbing Robots" — CLAWAR 2024 — arXiv:2409.13218 — 每 gripper 力传感器导纳控制抑制扰动反力、防**意外**脱附;仅 ClimbLab 仿真(HubRobo 模型) — 08-24"最近对标"判断成立且更好划界:闭环 F/T vs 开环、防意外 vs 主动放气瞬间、仿真 vs 实机 n=3、无滑移量化;其 21 refs+4 citations(全过目)无脱附前内力再分配。
- 🟠 Ribeiro, Uno, Imai, Murase & Yoshida (2023) "RAMP: Reaction-Aware Motion Planning of Multi-Legged Robots for Locomotion in Microgravity" — ICRA 2023 — arXiv:2301.07996 — 前馈规划低反力摆动轨迹防 gripper 意外脱附(气浮实验) — 同为前馈无力反馈,但管理摆动惯性反力,非静态储存弹性内力。
- 🟠 Murphy, Kute, Mengüç & Sitti (2011) "Waalbot II: Adhesion Recovery and Improved Performance of a Climbing Robot using Fibrillar Adhesives" — IJRR 30(1) — DOI 10.1177/0278364910382862 — 磁锁踝+槽式杠杆被动剥离机构以不均匀加载低力剥离 — 机构级免传感剥离力最小化;无机体位姿扰动量化。
- 🟠 Parness et al. (2017) "LEMUR 3: A limbed climbing robot for extreme terrain mobility in space" — ICRA 2017 — DOI 10.1109/ICRA.2017.7989643 — 每肢末端单轴力传感器管理附着/脱附力 — 传感器闭环脱附力管理的代表平台。
- 🟠 Yoshida & Ma (2010) "Design of a wall-climbing robot with passive suction cups" — IEEE ROBIO 2010 — DOI 10.1109/ROBIO.2010.5723554 — 被动吸盘拉缘剥离低力脱附 — 被动吸盘阵营的"机械泄压"思路(与"慢放气"同精神的机械版);无滑移量化。
- 🟡 Wang, Wang, Song, Zong, Zhang, Ji, Manoonpong & Dai (2023 (?)) "A Neural Coordination Strategy for Attachment and Detachment of a Climbing Robot Inspired by Gecko Locomotion" — Cyborg and Bionic Systems 4:0008 — DOI 10.34133/cbsystems.0008 — CPG+滞后延迟线协调气电混驱吸放时序(旋转脱附降拉脱力) — 脱附平顺化的协调/结构路线;年份 2022/2023 来源不一(§7)。
- 🟡 Xiao et al. (2024) "MST-G: Micro Suction Tape Gripper Climbing Robot with Active Detachment Capability" — Sensors 24(23):7790 — DOI 10.3390/s24237790 — 微吸胶带+线性电机机构级主动脱附 — "主动脱附"是使能机构非扰动缓冲;FTFOF 同组前作。
- 🟡 Spenko, Haynes et al. (2008) "Biologically inspired climbing with a hexapedal robot"(RiSE) — J. Field Robotics — DOI 10.1002/rob.20238 — feed-forward 循环步态+按表面调参的附着/脱附轨迹整形 — 开环轨迹整形先例(足端局部),无内力核算与滑移指标;是否量化每周期滑移未逐页核 (?)。
- 🟡 Shao, Chen, Zang & Li (2026) "Slip-Adaptive Neural Control of Gecko-Inspired Adhesive Robots" — Advanced Intelligent Systems — DOI 10.1002/aisy.202501168 — 关节力矩本体感知估计粘附力、检测/恢复滑移 — 口径是站立相粘附失效滑移,非释放瞬间;感知闭环对照面。

## 2. B 群:行走机力再分配经典带 → 现代 WBC(力域执行的证明链)

核心结论:1986 起"抬腿前把该腿力平滑卸到零"是行走机经典课题,但整条带
到现代 WBC/MPC **全部在力域执行**(力/力矩传感器闭环,或把力设定值交给
液压/力矩电机等可跟踪力的驱动层);"位置舵机+离线标定位移表纯开环清零
内力"无先例。

- 🟠 Waldron (1986) "Force and motion management in legged locomotion" — IEEE J. Robotics and Automation RA-2(4):214–220 — 行走机力/运动协调奠基(ASV 背景) — 经典带源头引用。
- 🟠 Kumar & Waldron (1988) "Force distribution in closed kinematic chains" — IEEE J. Robotics and Automation 4(6):657–664 — 多腿支撑=闭运动链,分解平衡力+相互(内)力 — 本文"内力"概念先祖;其内力是刚体静不定变量,我们的源于实测弹性且位移域操纵。
- 🟠 Kumar & Waldron (1990) "Force distribution in walking vehicles" — ASME J. Mechanical Design 112(1):90–99 (?) — 力分配在步行车辆步态(含腿起落事件)的应用 — ⚠ 正文被 403 挡,是否明写"抬腿时刻力恰为零"未核(§7),引用勿代宣称。
- 🟠 Klein & Kittivatcharapong (1990) "Optimal force distribution for the legs of a walking machine with friction cone constraints" — IEEE T-RA 6(1):73–85 — 摩擦锥约束下最优力分配 — 经典带必引;全力域。
- 🟠 Gorinevsky & Shneider (1990) "Force Control in Locomotion of Legged Vehicles over Rigid and Soft Surfaces" — IJRR 9(2):4–23 — 在纯位置控制六足机上加装力反馈实现力控行走 — 直接佐证"位置控制多足机管接触力的经典解=加力传感器闭环",反衬我方路线。
- 🟠 Chen, Cheng, Yang, Kung & Sun (1999) "Optimal force distribution in multilegged vehicles" — Robotica 17(2):159–172 — Compact-QP 力分配+支撑↔摆动切换期平滑段消指令力不连续,摘要明言平滑方案"does not require force sensors" — ⚠ 措辞陷阱:免传感器的只是**设定值计算**,执行仍需力跟踪驱动层;刚体框架、无弹性储能、无吸附。
- 🟠 Zhou, Low & Zielinska (2000) "An efficient foot-force distribution algorithm for quadruped walking robots" — Robotica 18(4):403–413 — FriCoM 法,爬行步态全程力分配连续 — "力过渡连续性"诉求代表。
- 🟠 Erden & Leblebicioğlu (2007) "Torque Distribution in a Six-Legged Robot" — IEEE T-RO 23(1):179–186 — 六足支撑腿力矩域 QP 分配 — 六足现代经典;全驱动力矩执行。
- 🟠 Görner & Hirzinger 等 (2009) "The DLR Crawler: evaluation of gaits and control of an actively compliant six-legged walking robot" — Industrial Robot — DOI 10.1108/01439910910957101(卷期未核 (?);IROS 2008 姊妹篇) — 关节力矩传感器+柔顺控制,明确"柔顺控制相比纯关节位置控制降低了内力" — 直接说出本文要划界的命题:他们靠力矩传感器消内力,我们靠标定表开环消。
- 🟡 Cheng & Orin (1990) "Efficient algorithm for optimal force distribution—the compact-dual LP method" — IEEE T-RA 6(2):178–187 — 高效 LP 求解(解随时间可跳变,是"连续性"工作的靶子) — 背景。
- 🟡 Marhefka & Orin (1998) "Quadratic optimization of force distribution in walking machines" — IEEE ICRA 1998:477–483 — QP 力分配最小化电机功率 — 背景。
- 🟡 Gardner (1991/1992) "Force distribution in walking machines over rough terrain" 等 — ASME J. Dyn. Sys. Meas. Control 113(4):754– / Robotica 1992(卷期未核 (?)) — 不平地面任意接触法向的高效力分配 — 经典带补全。
- 🟡 Song & Waldron (1989) "Machines That Walk: The Adaptive Suspension Vehicle" — MIT Press 专著 — OSU ASV 六足系统性专著 — 总背景;液压执行细节未从一手核实(§7)。
- 🟡 Sentis, Park & Khatib (2010) "Compliant Control of Multicontact and Center-of-Mass Behaviors in Humanoid Robots" — IEEE T-RO — virtual linkage 显式建模/控制多接触内张力 — 内力调节概念源头之一;全身力控。
- 🟡 Righetti, Buchli, Mistry, Kalakrishnan & Schaal (2013) "Optimal distribution of contact forces with inverse-dynamics control" — IJRR 32(3):280–298 — DOI 10.1177/0278364912469821 — 力矩冗余解析最优化接触力/内力 — 现代力域内力管理理论代表。
- 🟡 Focchi, del Prete, Havoutis et al. (2017) "High-slope terrain locomotion for torque-controlled quadruped robots" — Autonomous Robots 41:259–272 — DOI 10.1007/s10514-016-9573-1 — 50° V 坡支撑力最优分配防滑 — 大坡度最接近上墙工况的力域分配。
- 🟡 Bellicoso, Gehring, Hwangbo, Fankhauser & Hutter (2016) "Perception-less terrain adaptation through whole body control and hierarchical optimization" — IEEE-RAS Humanoids 2016:558–564 — ANYmal 分层 QP,支撑切换平滑 — 现代 WBC 接触过渡代表。
- 🟡 Di Carlo, Wensing, Katz, Bledt & Kim (2018) "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control" — IROS 2018:7440–7447 — DOI 10.1109/IROS.2018.8594448 — 凸 MPC 规划地反力序列(接触相起止力时序) — "接触力按计划渐变"现代代表;力矩控平台。
- 🟡 Bretl (2006) "Motion Planning of Multi-Limbed Robots Subject to Equilibrium Constraints: The Free-Climbing Robot Problem" — IJRR 25(4):317–342 — 平衡约束多肢自由攀爬规划鼻祖 — 规划层背景;不触脱附瞬态。
- 🟡 Shirai, Lin, Schperberg, Tanaka, Kato, Vichathorn & Hong (2022) "Simultaneous Contact-Rich Grasping and Locomotion via Distributed Optimization…" — IROS 2022 — arXiv:2207.01418 — ADMM 抓握+运动一体规划,45° 实机 — 规划层接触力分配。
- 🟡 Caccavale et al. (2008 (?)) "Six-DOF Impedance Control of Dual-Arm Cooperative Manipulators" — IEEE/ASME TMECH — DOI 10.1109/TMECH.2008.2002816 — 双臂协调内力阻抗 — 2409.13218 内力概念谱系来源;作者序未逐一核 (?)。
- 🟡 Dallmann et al. (2017) "A load-based mechanism for inter-leg coordination in insects" — Proc. Royal Society B 284(1868):20171755 — https://royalsocietypublishing.org/rspb/article/284/1868/20171755 — 竹节虫邻腿加载使本腿机械卸载,卸载信号(campaniform 感受器)触发摆动 — 生物学版"先卸载再抬腿"(靠载荷感受反射),给零力交接漂亮的生物动机+反衬无传感开环。
- 🟡 Kurazume, Yoneda & Hirose (2001/2002 (?)) "Feedforward and feedback dynamic trot gait control for quadruped walking vehicle" — ICRA 2001 / Autonomous Robots — DOI 10.1023/A:1014045326702 — sway compensation:位置域预移身体/ZMP 转移支撑载荷 — 划界句:"位移预铺转移**重力载荷**是静步态传统;过约束吸附系统中位移预铺清零**弹性内力**是新命题"。

## 3. C 群:位移域标定前馈补柔性(方法同构、目标不同)

- 🟠 Doi, Hodoshima, Hirose, Fukuda, Okamoto & Mori (2005) "Development of a quadruped walking robot to work on steep slopes, TITAN XI (walking motion with compensation for compliance)" — IROS 2005 — DOI 10.1109/IROS.2005.1545498 — 7 吨液压斜坡四足:预先人工标定腿系柔性,行走中前馈补偿弯曲变形恢复落足精度 — **位移域+离线标定+前馈的最近先例**;目标=位置精度非内力清零,无吸附无脱附;必引划界。
- 🟠 Ota, Kuga & Yoneda (2006) "Deformation compensation for continuous force control of a wall climbing quadruped with reduced-DOF" — ICRA 2006:468–474 — DOI 10.1109/ROBOT.2006.1641755 — 减自由度鼓风机吸附四足,补偿重力压弯机体的"补偿运动"求动作顺滑(摘要级,正文未取 §7) — **吸附平台上变形补偿的唯一先例**;补静态重力变形,不涉脱附瞬态/内力归零/放气时序;必引精确划界。
- 🟠 Wang, Zhang & Fuhlbrigge (2009) "Improving Machining Accuracy with Robot Deformation Compensation" — IROS 2009 — DOI 10.1109/IROS.2009.5353988 — 关节刚度模型+前馈修正指令轨迹补加工力挠度 — 方法学同构最经典先例(离线刚度模型→纯位置前馈);连续加工力非接触切换内力。
- 🟠 Klimchik, Bondarenko, Pashkevich, Briot & Furet (2014) "Compliance error compensation in robotic-based milling" — LNEE — arXiv:1409.6231 — 非线性刚度模型+离线轨迹修改 — 同族,给"标定+前馈补柔性"成熟谱系。
- 🟠 本田专利 Takenaka 等 (2005) "Floor shape estimation system of legged mobile robot" — US6922609/US6920374 — https://patents.google.com/patent/US6922609 — 腿式机器人 deformation compensation:前馈抵消柔顺机构/足底弹性变形 — 腿足平台前馈补挠度直接先例;变形量来自模型+期望地反力(闭环稳定器供给),且为专利非论文。
- 🟠 Wang, Weng, Wang, Wang, Wang, Dai & Jusufi (2024) "Wall-Climbing Performance of Gecko-inspired Robot with Soft Feet and Digits enhanced by Gravity Compensation" — Bioinspir. Biomim. — DOI 10.1088/1748-3190/ad5899 / arXiv:2405.02639 — 腿刚度模型+前馈重力补偿(QP 足力→指令位置),成功率 3/10→10/10 — 爬壁平台"刚度模型+前馈位置修正"先例;目标=体姿态/吸附角,不触脱附卸载与滑移。

## 4. D 群:平台谱系与替代路线(背景引用池)

**负压/吸盘阵营**(结论:无人量化释放瞬间身体滑移,无放气时序补偿先例;
对策只有"不脱附"与"压力闭环冗余"两类):
- 🟡 Hirose, Nagakubo & Toyama (1991) "Machine that can walk and climb on floors, walls and ceilings"(NINJA-I) — ICAR 1991 — ieeexplore 240585 — 并联连杆腿+VM 阀式多吸盘 — 腿式真空鼻祖;阀为"维持吸附"服务,脱附瞬态未见(正文未取 (?))。
- 🟡 Hirose & Kawabe (1998) "Ceiling walk of quadruped wall climbing robot NINJA-II" — CLAWAR 1998(无 DOI) — 天花板行走 — 正文不可得 (?)。
- 🟡 Nishi (1988/1992) 吸盘双足爬壁系列 — ISARC 1988(DOI 10.22260/isarc1988/0065)/ Mechatronics 2(6) — 经典平台,无脱附瞬态量化。
- 🟡 Zhang, Zhang, Zong, Wang & Liu (2006) "Sky Cleaner 3" — IEEE RAM 13(1) — DOI 10.1109/MRA.2006.1598051 — 气缸+吸盘组交替擦窗 — 摘要级未见释放滑移讨论 (?)。
- 🟡 Xiao & Sadegh (2007) "City-Climber" — InTech — DOI 10.5772/5090 — 不完全密封转子负压轮式 — 以"不需完美密封"回避破裂问题的路线。
- 🟡 Hillenbrand, Schmidt & Berns (2008) "CROMSCI" — Industrial Robot 35(3) — DOI 10.1108/01439910810868552 — 七可控负压腔+压力闭环对抗泄漏瞬变 — "传感器+阀时序"对策代表;对象是持续泄漏非步态脱附。
- 🟡 Longo & Muscato (2004) "Alicia3" — Industrial Robot 31(2) — DOI 10.1108/01439910410522838 — 滑动吸盘三模块 — 滑动负压代表。
- 🟡 Miyake & Ishihara (2006/2009) WallWalker 系列 — ISARC 2006 / IROS 2009(ieeexplore 4913279) — 滑动吸盘+液封润滑 — 日系滑吸代表。
- 🟡 Yue, Bloomfield-Gadêlha & Rossiter (2024) "Snail-inspired water-enhanced soft sliding suction for climbing robots" — Nature Communications 15 — DOI 10.1038/s41467-024-48293-2 — 水润滑滑吸,吸着不解除即可滑移;**引言明确列举离散步态"反复破坏再重建吸附"的缺点** — 问题动机引用+回避路线对照(需持续供能/供水)。
- 🟡 Kim, Kim, Yang, Lee 等 (2008) "Development of a wall-climbing robot using a tracked wheel mechanism" — J. Mech. Sci. Tech. 22 — DOI 10.1007/s12206-008-0413-x — 履带链 24 吸盘、机械阀按位置顺序吸/放 — "机械阀时序放气"硬件先例(连续履带,无滑移量化)。
- 🟡 福田敏男ほか (1992)「吸盤装着クローラ形壁面走行ロボット」 — 日本機械学会論文集 C 編(卷期未核 (?)) — 导管+机械阀自动开闭履带式 — 日系早期代表。
- 🟡 Wang, Wang, Zong & Li (2010) "Principle and experiment of vibrating suction method" — Vacuum 85(1) — DOI 10.1016/j.vacuum.2010.04.010 — 振动吸附持续再生真空 — 吸附维持路线。
- 🟡 Shi, Xu, Xu & Jiang (2022) 6-DOF 人形负压双足 — Mechatronics 87 — DOI 10.1016/j.mechatronics.2022.102889 — 近年脚式负压平台(摘要不可得 (?))。
- 🟡 Wang, Bao, Zhang & Yang (2009) 柔性气动爬壁 — J. Cent. South Univ. Tech. 16 — DOI 10.1007/s11771-009-0160-x — "大柔性腿链+吸盘"同类平台,未见脱附处理 (?)。
- 🟡 潘雷, 赵言正, 钱志源, 付庄, 曹其新 (2005) "具有双负压吸盘的爬壁机器人吸附特性" — 上海交通大学学报 39(6):873–876 — 流体网络模型给风机突启/遇障/突停三工况**腔压动态响应**并实验验证 — 中文最接近"释放/失效瞬态"的定量工作;口径是腔压非本体位移。
- 🟡 管贻生组 (2010/2013) "W-Climbot: A modular biped wall-climbing robot" — IROS 2010(ieeexplore 5589064)/ IEEE-ASME TMECH 2013 — 5 关节模块+双端负压尺蠖双足 — 国内最接近本文形态的经典系列;未量化脱附滑移(全文未逐页核 (?))。
- 🟡 Li, Zhang, Huang…Xu (2025) "Development of a New Biped Robot With Adaptive Suction Modules for Curved-Surface Climbing" — IEEE RA-L(ieeexplore 11027566;同组 T-ASE 2025 DOI 10.1109/TASE.2024.3390030) — 澳门大双足负压曲面攀爬 — 同刊同吸附方式最新平台,宜引。
- 🟡 王斌锐等 (2014) "曲面上双足三自由度爬壁机器人设计与稳定性分析" — 机器人 36(3) — DOI 10.3724/SP.J.1218.2014.00349 — 防倾覆/防滑落**静力**约束 — 代表中文"滑落=静力失稳判据"通行口径,反衬本文逐周期动态位移口径空白(同组 2018 智能系统学报同 (?)核实结论)。
- 🟡 吉田壱平ほか (2023)「壁面移動を目的とした吸盤脚型ロボット『GeckoPus』の開発」 — ROBOMECH 2023 — DOI 10.1299/jsmermd.2023.2P2-G13 — 吸盘脚型四足 — 日系近年;概要级无脱附内容。另:Kawasaki & Kikuchi (2014) 被动吸盘六足(venue 未核 (?))/ 清水・菊池 (2019) JSME 関東支部 19A15(全文已读;其"滑移"=转弯运动学滑移);Yamasaki et al. (2015) ROBOMECH(DOI 10.1299/jsmermd.2015._2a2-o10_1)。
**干粘附/钩爪/抓握平台**:
- 🟡 Unver, Uneri, Aydemir & Sitti (2006) "Geckobot" — ICRA 2006 — DOI 10.1109/ROBOT.2006.1642050 — 弹性体粘附四足+剥离机构+主动尾 — 动机段含"吸盘需大脱附力低效 vs 弹性体低力剥离"的最早一句话级对比(引语出自检索摘要 (?))。
- 🟡 Henrey et al. (2014) "Abigaille-III" — J. Bionic Engineering — 六足双层干粘附 — 同构型不同物理对照。
- 🟡 Birkmeyer, Gillies & Fearing (2012) "Dynamic climbing of near-vertical smooth surfaces"(CLASH) — IROS 2012(ieeexplore 6385775) — 远心踝减剥离力矩,4–12Hz 动态爬 70° — 靠机构+步频让脱附"来不及"扰动。
- 🟡 Provancher, Jensen-Segal & Fehlberg (2010) "ROCR" — IEEE/ASME TMECH(ieeexplore 5546974) — 摆锤双钩爪动态爬地毯墙 — 钩爪脱附零扰动是钩子物理的免费午餐(对照:该问题在钩爪线不存在)。
- 🟡 Yu 等 (2024) 变刚度粘附爪(微重力) — Advanced Intelligent Systems — DOI 10.1002/aisy.202400043 — 材料/机构级脱附力最小化(摘要级 (?))。
- 🟡 Uno et al. (2021) "HubRobo" — IEEE-RAS Humanoids — DOI 10.1109/HUMANOIDS47582.2021.9555799 — 被动 spine gripper 四足 — 平台背景。
- 🟡 Tanaka et al. (2022/2023) "SCALER" — IROS 2022 / 期刊版 — arXiv:2207.01180 / 2312.04856 — GOAT 欠驱动 gripper 攀爬 — 平台背景。已核不占位:Uno et al. (2026) LIMBERO(arXiv:2603.16531,摘要无脱附/内力内容)。
**综述**:
- 🟡 Nansai & Mohan (2016) — Robotics 5(3) — DOI 10.3390/robotics5030014;Tao, Gong & Ding (2023) "Climbing robots for manufacturing" — NSR 10(5) — DOI 10.1093/nsr/nwad042(无脱附滑移专节);马吉良等 (2023) 机械工程学报 59(5) — DOI 10.3901/JME.2023.05.011(全文核:无);**裴香丽…戴振东 (2023) 科学通报综述(全文核:点名"瞬态力…机体震颤"引 Stickybot、"减小脱附冲击"列为待攻关——动机引用+中文侧负证据双重价值)**;MDPI Robotics 11(6):143 (2022) gecko 综述(含吸盘大脱附力定性对比);Electronics 14(14):2810 (2025) 综述。

## 5. E 群:对照物理与传感替代路线

- 🟠 Tang, Chi, Sun 等 (2020) "Leveraging elastic instabilities for amplified performance: spine-inspired high-speed and high-force soft robots" — Science Advances 6:eaaz6912 — arXiv:1810.08571 — snap-through 双稳态数十毫秒快存快放弹性能驱动爬行 — **同一物理、相反方向**的旗舰对照:他们利用瞬释推进,我们把瞬释认定为寄生扰动并预先泄掉。配套:双稳态跳跃器(2024,Science Robotics,DOI 10.1126/scirobotics.adm8484,<15ms 释能起跳,作者未核 (?))。
- 🟠 Wahrburg, Bös, Listmann, Dai, Matthias & Ding (2018) "Motor-Current-Based Estimation of Cartesian Contact Forces and Torques…" — IEEE T-ASE(ieeexplore 7914641) — 电流+运动学 Kalman 估计接触力 — "无力传感器估力"替代路线代表;依赖精确摩擦/动力学模型,在廉价舵机+大摩擦腿链上不可行——本文开环路线的动机引用(回答"为何不用电流闭环")。
- 🟠 (作者未核 (?)) (2025) 蠕动机器人连续体模型 — J. Mech. Phys. Solids(sciencedirect S0022509625000109) — 干摩擦致逐周期 backward slippage 及速度折损的建模预测 — "每周期位置损失"已在蠕动谱系命名量化;机制=摩擦锚定不足,非弹性储能瞬释。
- 🟡 Rafsanjani et al. (2018) "Kirigami skins make a simple soft actuator crawl" — Science Robotics 3:eaar7555 — 各向异性摩擦"棘轮"单向爬行 — "ratchet"正向用法的术语对照。
- 🟡 Endlein & Federle (2013) "Rapid preflexes in smooth adhesive pads of insects prevent sudden detachment" — Proc. Royal Society B — DOI 10.1098/rspb.2012.2868 — 昆虫粘附垫 1ms 被动扩大接触防意外瞬脱 — 生物侧连"意外 snap 脱附"都有专门被动机制防,反衬主动破裂式脱附必须工程化管理。
- 🟡 Tian, Pesika, Zeng…Autumn, Israelachvili (2006) "Adhesion and friction in gecko toe attachment and detachment" — PNAS 103(51) — DOI 10.1073/pnas.0608841103;Autumn et al. (2006) "Frictional adhesion: a new angle on gecko attachment" — J. Exp. Biology 209:3569 — 壁虎剥离角力学、~20ms 快速低力脱附 — 进化解决了"快且低扰脱附";干粘附机器人继承此物理,**真空吸盘无法继承——只能在破裂前把能量先归还**(讨论节好句)。
- 🟡 (作者未核 (?)) (2023) "Modeling multi-legged robot locomotion with slipping and its experimental validation" — arXiv:2310.20669 — 地面多足滑移建模 — 背景。
- 🟡 Pratt & Williamson (1995) "Series Elastic Actuators" — IROS 1995 — 柔性储能通用背景;SEA 谱系未见专门量化"接触断开瞬间储能释放扰动"(倾向不存在,假肢/外骨骼分支未扫)。

## 6. Related Work 节写作骨架(段落级)

1. **平台谱系一句带过**:吸附方式分真空/负压、干粘附、抓握/微刺三线
   (各引 2–3 代表+综述 Nansai16/Tao23);本文属腿式负压线(NINJA→
   Sky Cleaner→W-Climbot→澳门大 RA-L25)。
2. **脱附扰动的既有认识**:定性预告(Stickybot 2008 瞬态传播;科学通报
   2023"机体震颤/待攻关")→ 力/振动量化簇(Wang21 剥离角阻抗、Wen25
   旋转剥离 −65.5%、FTFOF 力 −28%/振动 −82%、Waalbot II/Yoshida&Ma
   机构剥离、MST-G)→ 空白:无人量化**整机位置损失**,真空破裂通道仅有
   腔压量化(潘雷 05)。⚠ 点破 −82% 量纲巧合。落点=贡献①。
3. **脱附前卸载思想谱系**:Stickybot 专利(力域闭环首述)→ LORIS Unload
   步(在线力优化)→ LEMUR3/Imai24(传感器管理)→ RAMP(前馈但管惯性
   反力)→ Boscariol13(准静态规划)。全部力域/闭环/规划层;本文=位移域
   +离线标定+开环+实机滑移验证。落点=贡献③收窄措辞。
4. **行走机力再分配经典带→现代 WBC**:Waldron86/K&W88/Klein90 起,
   Chen99(注意其"免传感器"只指设定值)、Zhou00、Erden07,到 Righetti13/
   Focchi17/Bellicoso16/DiCarlo18——**全在力域执行**;DLR Crawler 明言
   位置控制积累内力、其解=力矩传感器。Dallmann17 昆虫"邻腿加载卸载后才
   摆动"作生物通则注脚(靠载荷感受器;我们无传感开环)。落点=位移域开环
   是该经典问题在"位置舵机+强柔性"平台上的新解。
5. **位移域前馈同手段划界**:TITAN XI(标定柔性→落足精度)、Ota06(吸附
   平台补重力静变形)、铣削挠度补偿(Wang09/Klimchik14)、本田专利、
   sway compensation(重力几何转移)、Jusufi24(姿态/吸附角)——同数学
   形态,无一用于"脱附前内力清零",无一处理弹性储能瞬释。
6. **替代路线收束**:滑动吸附回避脱附(Yue24,代价=持续供能);电流估力
   闭环(Wahrburg18,廉价舵机不可行);慢放气(本文 08-20 数据否决);
   snap-through 反向利用(Tang20)。→ 本文定位:零传感器开环、软件改装、
   现象-模型-解法闭环。

## 7. 引用前须人工核全文清单(按重要度)

1. **Kumar & Waldron 1990**(ASME JMD)正文:是否明写"抬腿时刻力恰为零/
   平滑归零"——审稿人若熟正文,漏引其论述有风险;引用措辞前必核。
2. **FTFOF arXiv:2504.19448 正文**:开/闭环执行细节(PDF >10MB,读 HTML 版
   https://arxiv.org/html/2504.19448v2)——直接影响本文"唯一开环"措辞强度。
3. **西工大《一种爬壁机器人真空吸附性能分析与运动切换策略》**(机械科学
   与技术,journals.nwpu.edu.cn 瑞数反爬仅见题名)——题含"运动切换策略",
   有知网权限时人工复查。
4. **Ota ICRA 2006 正文**(变形补偿实现细节;目前摘要级)。
5. **Stickybot T-RO 2008 与 TITAN XI 的 citations 树**(S2 限流未跑):
   看是否有人已把"脱附前卸载/标定前馈"搬到吸盘或开环平台。
6. CLAWAR 2024 终版 Imai vs arXiv v1(是否补实机;概率低)。
7. CNKI 硕博全文(彻底排除中文量化先例;现状=三篇中文综述全文反查未
   收录,间接证据)。
8. 戴组 sciengine《仿壁虎机器人脚掌的黏附性能研究及模拟微重力下黏脱附
   轨迹设计》(DNS 不通未读,疑似"脱附轨迹设计"近亲 (?))。
9. Frontiers Robotics & AI 2022 "Adaptive robot climbing with magnetic feet
   in unknown slippery structure"(在线重分配期望地反力;见标题未开)。
10. 出处补全:Cyborg cbsystems.0008 年份(2022/2023)、Kawasaki&Kikuchi
    2014 venue、福田 1992 卷期、Görner DLR Crawler 卷期、山崎 2015 日文
    原题、Santos ICRA07 与 Geckobot 引语原文逐字、Waalbot II 卷期终核、
    双稳态跳跃器/JMPS 蠕动/2310.20669 作者名。
11. 低优先:ASV 专著液压执行细节、俄系(Devjanin/Gurfinkel)与德系
    (Schneider & Schmucker)力控线、LAURON 系列、ROCR 正文、假肢/外骨骼
    SEA 分支、Autumn 脱附能量学(如需生物脚注)。

## 8. 残留风险总评

- **CNKI 墙内是唯一无法完全排除的暗区**(中文硕博正文);缓解证据=三篇
  2021–2023 中文综述全文反查均未收录"脱附滑移量化"类工作,风险低。
  成文措辞统一用"据我们所知(to the best of our knowledge)"。
- S2 搜索端点整场限流,相关性排序缺失,冷僻标题存在极小漏检概率
  (已用 OpenAlex/Crossref/WebSearch 三路补位)。
- 老会议(CLAWAR 1998、ISARC)与部分正文不可得的平台论文里,可能存在
  一两句"释放时机体会微动"的**定性**描述——不影响"无量化+无开环卸载
  先例"结论,但"首次定性指出"这种话不要写。
