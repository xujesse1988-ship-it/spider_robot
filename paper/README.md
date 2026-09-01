# RA-L 论文草稿(T6)

状态:**英文全文初稿 v0(2026-08-31)**——全部章节成文、核心数字已嵌入,
待补图/待办见下。总纲 `docs/PAPER-PLAN.md`;本目录只放论文本身。

## 文件

- `main.tex` — 正文(IEEEtran journal 模式,RA-L 口径;红色 `\TODO{}` 为待办标记)
- `refs.bib` — 45 条引文(源自 `docs/RELATED-WORK.md` 判级表;
  条目内 `TODO(verify)` = 投稿前须人工核全文的字段,总清单在 RELATED-WORK §7)
- `figures/` — 插图(生成脚本与来源见各文件头/下表)
- `references/` — 参考文献 PDF,**文件名=refs.bib 引用键**(现有
  nadan2024loris.pdf=LORIS ICRA 2024,CMU 作者公开版——写作范本兼
  贡献③划界近邻;写作范本首选另见 kim2008smooth=Stickybot T-RO 2008,
  叙事弧同构:同一个敌人"脱附瞬态"、同款"脱开时刻本来就没力"哲学)

## 编译

开发机没装 LaTeX。两条路:

1. **Overleaf(推荐)**:上传 `main.tex` + `refs.bib` + `figures/`,
   编译器选 pdfLaTeX,IEEEtran 类 Overleaf 自带;
2. 本机装 `texlive-publishers`(含 IEEEtran)后:
   `pdflatex main && bibtex main && pdflatex main && pdflatex main`。

## 数字溯源(写作时改数必查源)

| 论文位置 | 数字 | 来源 |
|---|---|---|
| 摘要/主表 Table I | −43%/−61%,−82%/−88%,p=0.0013/0.00092/0.0022 | html/handover-ab-n3-20260826.html §1 |
| 头条口径 | 93%(n=3 A 组 vent 段 172.9/185.5);83%=08-20 首次发现口径 | 同上 + html/vent-snap-20260820.html(口径差异=分段窗定义,正文已注) |
| 破裂剖面 | 0.4s 泄压 <0.15mm;33~67ms 完成;L1 过冲 23.8→21.5 | html/vent-snap-20260820.html §4 |
| δ 标定 | 斜率 −0.57~−0.13;δ* 28~42;表 31/33/29/22/22/24;66→50→39mm | html/handover-delta-calib-20260824.html |
| 传递比 | 42.2±3.0%(9 标记/3 天/3 机位) | n=3 报告发现 2 |
| 模型拟合 | RMSE 0.85/CV 0.99;两路 k̂ r=0.938;B 60.2=6.7+55.5;C 39.7=7.7+30.0 | html/stiffness-fit-20260826.html |
| κ 定律 | Δb=κ·δ(1+0.20(r−1));κ_B 0.0988/κ_C 0.0550/κ_31 0.0529;权重压 44% | html/dprime-20260831.html(M10 AICc 0.0 vs M8 17.8;216 点) |
| D′ 配对 | D′−C 交接段 −2.9±2.1(p=0.14);总 −1.7±0.7;落点比例 0.21 | 同上 §1-2 |
| 联合 k̂ | L1 0.265/R1 0.227/L3 0.137/R2 0.133/R3 0.125/L2 0.112;γ=0.11 | 同上 §3 |
| 系统参数 | 吸附 ≤−30kPa;泵滞环 −60/−75;抬腿门槛 −50kPa | software/hexapod/adhesion.py, config.py(已核代码) |

## 待办(投稿前;`main.tex` 内红字与此同步)

1. **作者/单位/邮箱/资助**(main.tex 头部)。
2. **整机称重**——Sec III-A 的 ~2.7kg 是估计,P4-GUIDE 验收表本就欠着这项。
3. **图**:Fig.1 平台照片+交接示意(要拍/画);Fig.2 阶梯轨迹+破裂剖面
   (T3,`ab_quant --zoom`);Fig.3 弹跳-δ 线性;Fig.4 分解瀑布+逐腿弹跳;
   Fig.5 D′ 对齐剖面。figures/ 里已生成的见文件头注释,其余 `\TODO`。
4. **vent 窗定义核对**(Sec III-B 的 −1s/+0.4s 界,从 ab_quant 源码确认后定稿)。
5. **T4a 挂重**(绝对 k+蠕变率直测)与 **T4b 净前进率 demo**(26%→X%)
   结果出来后并入 Sec VII(讨论已留钩子);26% 基线数字当前来自 08-19。
6. **投稿视频**:A vs C 并排 + 破裂慢放(MOV 原片冻结在本机,协议见 PAPER-PLAN T6)。
7. **开源数据包**:日志/traj/report/188 张核对图/管线脚本打包,定托管处与许可证。
8. **refs.bib 逐条核全文**(`TODO(verify)` 字段 + RELATED-WORK §7 清单:
   Kumar&Waldron 1990 措辞、FTFOF 开/闭环细节、Ota 2006 正文、CNKI 硕博)。
9. **篇幅**:RA-L 8 页含引文;当前草稿约 9~10 页量级,定稿时砍
   (候选:Related Work 六段压四段、P1-P4 合并表述、偏差清单压缩)。
10. 术语统一自查:vent-snap ratchet / zero-force handover / handover segment /
    rupture segment / transfer ratio / per-mm transfer loss(κ)。

## 措辞红线(T5 定界,动 Related Work/贡献段前必读)

- ❌ 不写"首次提出脱附前卸载"(Stickybot 专利 US7762362B2 + LORIS Unload 步在先);
  ✅ 只主张:位移域换算 + 零传感器纯开环 + 破裂段滑移量化验证。
- FTFOF 的 −82% 是振动(m/s²),与我们 −82%(mm)量纲不同——正文已点破,勿删。
- 发现 2(42% 传递比)写"平台特性表征",不写"新物理"。
- 所有"首次"均带 "to the best of our knowledge"(CNKI 墙内是唯一未彻底排除的暗区)。
