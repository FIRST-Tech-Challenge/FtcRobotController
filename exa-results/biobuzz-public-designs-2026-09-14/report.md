# BIOBUZZ 公开机器人模型与早期机制讨论

抓取日期：2026-09-14（Asia/Shanghai）  
检索窗口：重点检查 2026-09-12 Kickoff 后至 2026-09-14 的公开内容  
范围：公开网页、Chief Delphi、Reddit、YouTube/FUN Robotics、GitHub、Onshape/厂商资源。登录后 Discord、部分 Instagram/TikTok 内容不在可验证范围内。

## 结论先行

1. **真正公开、可直接复制/量尺寸的完整 CAD，目前确认到的主要是 REV 的官方 Starter Bot Onshape。** 本轮没有确认到参赛队在 Kickoff 后公开完整、可复制的 BIOBUZZ 整机 CAD。
2. **队伍公开内容已经很多，但主要是 Robot in 30 Hours 的实机原型与访谈，而不是 CAD 文件。** FUN 的集中帖目前是最有价值的入口。
3. 早期设计明显分成两条路线：
   - **POLLEN 专用高速循环**：简单进球、单一压缩量、稳定射球，优先 HIVE TIP/RP。
   - **POLLEN + NECTAR 全兼容**：通过被动浮动压缩、尺寸分流或两套射球路径解决 2.8/3.6 英寸球差异，并增加 FLOWER 放置能力。
4. 多支队伍在 30 小时内都暴露了相同瓶颈：不是“能不能射”，而是 **连续供球、左右居中、双尺寸压缩、球型识别、射速恢复和可重复命中**。

## 已确认的公开模型

| 来源 | 开放程度 | 机制 | 判断 |
|---|---|---|---|
| [REV 2026–27 Starter Bot 文档](https://docs.revrobotics.com/ftc-kickoff-concepts) / [完整 Onshape](https://cad.onshape.com/documents/9cb9eb90e973fd37d10ee178/w/b9c02b0f4e09998f08becbd2/e/a55c018c62564f3ea8812a47) | 完整公开，可测量、复制 | 地面收集 POLLEN、发射进 HIVE；侧面 servo poker 清 FLOWER；含 build guide、BOM、TeleOp/Auto | 当前最可靠的可落地基线，但属于厂商 Starter Bot，不是强队赛季终版 |
| [The Janitors 36721 机器人页](https://janitorsftc.com/) | 有 CAD 渲染和制造记录，未找到可复制 Onshape/STEP | 自研四模块 swerve；当前只覆盖底盘，不是 BIOBUZZ 上层机构 | 可参考模块设计过程，不能算完整赛季机器人模型 |
| [Lebob Robotics 29550 GitHub](https://github.com/Lebob-Robotics/Lebob-BioBuzz) | 公开代码仓库 | 页面承诺赛季中公开代码与设计笔记；抓取时最近更新时间仍在 Kickoff 前 | 暂时不是可用机械模型 |
| [Lancer Robotics BIOBUZZ GitHub](https://github.com/LancerRobotics/biobuzz) | 公开代码仓库 | FTC SDK/TeamCode 类仓库 | 未发现可验证的赛后机械 CAD 或成熟机制 |
| [KronBot BIOBUZZ GitHub](https://github.com/kronbot/biobuzz) | 公开 fork | 主要仍是 FTC SDK 基线 | 名称是 BIOBUZZ，但不能据此当作机器人方案 |

## 已确认的队伍原型与思路

集中入口：[FUN FTC BIOBUZZ / Robot in 30 Hours 内容帖](https://www.chiefdelphi.com/t/fun-ftc-biobuzz-content-thread-ri30h/524050)。该帖持续汇总原型短片、策略节目和 Georgia Tech 三队进度。

| 队伍 | 公开方案 | 已展示的问题/下一步 | 原始链接 |
|---|---|---|---|
| 9224 Techno G.O.A.T.S. | 可升降 pivot intake；boot rollers 把球送上柔性透明斜坡；计划双轮向后上方发射 | 当时只完成 intake 主体，升降电机、斜坡和 launcher 仍在装配 | [原型短片](https://youtube.com/shorts/vDd_NoH-QLo) |
| 13201 Team Hazmat | noodles + Gecko wheels 的柔性 intake，同时兼容两种球；bucket/pass-through；两杆 gate 单颗放入 FLOWER；shooter 装在滑轨上让 NECTAR 被动顶开获得不同压缩量 | 需要把双尺寸压缩、单颗出球和整条 transfer 真正联调；另一个版本用 belt + “golf-club” kicker 送入 launcher | [Day 1 详解](https://youtu.be/xXOKL8JuE88) · [Flower/pass-through](https://youtube.com/shorts/KAwayfhN5dQ) · [Transfer/kicker](https://youtube.com/shorts/sF3aWmby_ls) |
| 9225 Dynamite Social Club + 34032 Paradox | rubber-band intake → Gecko wheels → 双侧反向旋转 shooter；用一台 6000 RPM 电机、链条和齿轮带动两轮；servo 调仰角 | 球左右散布导致精度差，计划缩窄通道并增加居中轮；还在调出射角和一致性 | [Outtake/transfer](https://youtube.com/shorts/eb4_jLG7J9k) |
| 16460 GEarheads | surgical-tubing intake；servo flapper/indexer；两台 6000 RPM shooter motors；另一侧用 claw + wrist + vertical slide 放 FLOWER | 重点仍是 launcher 一致性和整机练习；节目中提到未来公开 subsystem models，但当时网站尚未上线 | [Flower dropper/outtake](https://youtube.com/shorts/5dlcGfkOLkk) |
| 26691 THUNDERHeads | 四个 compliant wheels 配飞轮配重，提高连续四球时的惯量；速度 P 控制补偿电池下降；servo gate 控制供球；另有会抖动的 stinger 一次清空 FLOWER | 原型展示了闭环速度与储能思路，但仍需验证多球连续发射后的恢复时间和命中率 | [Stinger/outtake](https://youtube.com/shorts/mkRDGOajRuM) |
| 9686 Raiders of the A.R.C. | 明确选择 POLLEN 专用高速循环；wedge intake、可调 hood、居中高惯量定制钢飞轮；尝试 bang-bang 速度控制；后续设想 turret 与锁轮 PTO 防守 | 暂时放弃 NECTAR 与 FLOWER，换取简单和一致；队伍报告在一个固定场上参考位置测试约 2000 RPM，但这不是可直接照搬的通用设定 | [初始策略](https://youtu.be/iSao4o4rmSE) · [Day 2 更新](https://youtu.be/bR91YppIwNk) |
| 11347 Circuitrunners Blackout | 用高度挡板让小 POLLEN 从下方通过，大 NECTAR 被斜面导向另一通道；两种球进入不同 shooter | 结构复杂度和体积明显上升；仍在单独测试 POLLEN 压缩量 | [初始策略](https://youtu.be/iSao4o4rmSE) |
| 13245 Arcturus Apollo | POLLEN 专用 intake；入口挡杆拒绝 NECTAR；3D 打印带挡片 conveyor 约束球路再送 shooter | 是有意的专用化，而不是全兼容；需要与联盟分工一起评估 | [Day 2 更新](https://youtu.be/bR91YppIwNk) |
| Georgia Tech Ri30H 三队 | 11347、9686、13245 同场做 30 小时原型和最终小型 scrimmage | 适合观察机制如何从白板方案收敛为能跑的机器人 | [活动帖](https://www.chiefdelphi.com/t/ftc-biobuzz-robot-in-30-hours-georgia-tech/523993) |
| 13896 Battle Stallions | 公开 45 分钟 Robot in 24 Hours 构建视频 | 可确认活动与视频存在；本轮抓取未拿到可靠转录，暂不从画面猜机构细节 | [完整视频](https://www.youtube.com/watch?v=Oz46U6bkxy8) |
| 16169 Banana Box | 公开 Robot in 24 Hours 分段进度 | Hour 3 页面只说明持续搭建、试验和调整，缺少可核验技术细节 | [Hour 3](https://www.youtube.com/watch?v=ul4K3svu5Ug) |
| Ragnarok FB | 发布 9 秒 shooter test | 只能确认射球测试存在，缺少尺寸、压缩量、转速和命中数据 | [测试短片](https://www.youtube.com/watch?v=99Xht1nxa-c) |

## 社区讨论正在收敛的方向

### 1. 先专用还是先全能

Ri30H 中最明确的分歧不是 turret，而是 **是否在第一版就处理 NECTAR/FLOWER**。9686 与 13245 主动拒绝大球，只优化 POLLEN；13201、11347 则分别尝试被动变压缩和尺寸分流。30 小时原型显示，全兼容会同时增加 intake、transfer、shooter 与软件状态机复杂度。

### 2. 双尺寸兼容有三条现实路径

1. **柔性 intake + 被动浮动 shooter**：13201 的方向；零/少执行器，最值得先做台架。
2. **按尺寸分流至两套球路/射手**：11347 的方向；更可控，但占空间且消耗电机/结构预算。
3. **只做 POLLEN**：9686/13245 的方向；最容易先获得稳定循环，但会把 NECTAR 与 FLOWER 价值交给联盟伙伴。

### 3. Shooter 的共同技术重点

- 高惯量飞轮帮助多球连发时少掉速，但会拉长启动时间并增加重量。
- 只给“电机功率”不够；已有队伍在用 velocity P 或 bang-bang 控制，目的都是抵消电池电压与连续发射造成的转速波动。
- 供球 gate/indexer、左右居中和固定球路比单纯提高 RPM 更早成为命中率瓶颈。
- turret/可调 hood 能扩大射击位置，但第一版往往先固定或只调仰角，等基础弹道稳定再增加自由度。

### 4. FLOWER 目前更像独立子系统

公开原型普遍把 FLOWER 作为独立 claw/lift/dropper 或 descore stinger，而没有把高速 HIVE shooter 强行复用成精确放置机构。这与“高速射球”和“慢速、接近竖直放置”所需控制特性不同有关。

### 5. 视觉方向已经出现，但仍是概念验证

13201 公开讨论了三种用途：场上追踪 POLLEN/NECTAR、识别机器人内部 bucket 里的球型以选择压缩状态、判断 HIVE 哪一侧朝上。其团队已用自建数据集替代单纯色块检测，但未公开足够数据证明全场鲁棒性。

## 对 FTC-20827 BioBuzz 最值得立即复现的四个测试

1. **双尺寸被动压缩台架**：固定 shooter 轴，另一轴做小行程滑轨/弹簧浮动；测 POLLEN 与 NECTAR 的压缩量、射程、散布和卡球率。
2. **四球连发转速恢复**：记录每次出球前后 RPM、恢复到目标转速所需时间；比较轻飞轮、加配重飞轮、P/PIDF 与 bang-bang。
3. **入口自居中与墙角取球**：比较 full-width noodles、Gecko/compliant wheels、侧导板和 pivot intake；必须覆盖单球、成堆、贴墙、角落、两种球混合。
4. **先分离 FLOWER 原型**：做一个最小的 tube/bucket + gate 或 stinger，不要一开始就和 shooter 共用所有球路；分别测放置成功率、清空时间和对整机重心的影响。

建议第一轮不要直接选“全能整机”。先让两个小组并行：A 组验证 **浮动压缩是否能一套 shooter 打两种球**，B 组验证 **POLLEN 专用 intake → indexer → shooter 的四球连续循环**。数据决定是否值得承担尺寸分流、第二 shooter 或 FLOWER lift 的复杂度。

## 低置信度或被排除的结果

- Kickoff 前 Reddit 关于“REV 泄露 Onshape”的讨论相互矛盾；现在已有 REV 官方页面和官方 Onshape，应以官方版本为准，不再使用泄露链条做设计依据。
- 若网页只有“BIOBUZZ”“CAD”“高吞吐量”等营销文字，却没有日期、文件链接、视频或可测量模型，本报告不把它计为公开方案。
- GitHub 仓库以 BIOBUZZ 命名不代表已有赛季机构；若内容仍是 FTC SDK fork 或 Kickoff 前骨架，只列为观察对象。
- Discord、封闭团队服务器、需要账号的社媒内容没有被当成“已验证公开模型”。

## 检索说明

- Exa：12 次定向检索，共审阅 120 个返回结果，覆盖 CAD/Onshape、GitHub、队伍视频、Robot in 30 Hours、Reddit 与 Chief Delphi；随后抓取高信号视频页面/转录和主帖。
- 通用网页检索：用于验证 Chief Delphi 原帖、Reddit 当日讨论、REV 官方文档和 Onshape 链接。
- 大量结果是同一 FUN 内容帖的镜像、无技术细节的队伍主页、Kickoff 前猜测或与 FTC 无关的同名 “BioBuzz”，均已去重/排除。

