"""地-墙过渡（地爬墙，P5）——独立一份，不和其他功能代码混。

2026-09-13 从 hexapod/ 拆出来（用户："地爬墙的代码单独一份，不和其他的混合"）。
本包只依赖全机共用的底座：hexapod.config（尺寸/标定）、hexapod.kinematics（IK）、
hexapod.adhesion（真空状态机与 IO）、hexapod.driver / hexapod.robot（舵机）、
hexapod.runlog / hexapod.powerlog（黑匣子）。**不 import** hexapod.climb / gait /
scripts/climb_walk——原先借用的常量、站位半径求解、参数解析、状态行/收尾函数都
复制在 base.py / ui.py 里，那边改了这边不跟（这是有意的）。

  engine.py  MountEngine：每腿一个接触面、单腿跨面挪动、位姿铺设、零力交接/接管、
             最正落点搜索（原 hexapod/mount.py）
  base.py    从 climb.py / gait.py 复制来的常量、_solve_reach、parse_*、启动吸附序
  ui.py      从 climb_walk.py 复制来的状态行与阀线圈收尾
入口脚本仍是 scripts/mount_wall.py；测试 tests/test_mount.py；规划器 tools/mount_plan.py。
"""
from .engine import MountEngine, MountPhase, FLOOR  # noqa: F401
