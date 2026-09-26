# Git 使用指南（本项目）

> 日期：2026-09-22
> 起因：本项目此前没有版本管理，改坏了只能靠记忆和 `docs/` 里的日志还原。本文写给**不熟悉 git 的人**，目标是"会回退、会看历史、不怕改坏"。
> 当前约定（2026-09-26）：电脑负责 Git；WSL 可检查同一电脑工作区；树莓派仅接收完整源码包并校验，不要求安装 Git。
>
> GitHub 私有仓库：https://github.com/pineapple-miriam/sp_vision_25-main 。远程名为 `origin`；默认显示当前工作分支 `feat/calibration-web-reuse`，历史 `main` 和其他开发分支保留原样。
> `commit` 记录电脑本地版本，`push` 才把已提交版本上传；两者都不会自动更新树莓派。

---

## 0. 先记住三句话

1. **git 只会丢"你没提交的东西"。** 只要 `git commit` 过了，那条记录几乎永远找得回来（`git reflog` 是最后一道保险）。
2. **回退前先 `git status` 和 `git log --oneline -5`。** 90% 的误操作来自于不知道自己站在哪。
3. **不确定会不会丢数据时，先提交一次再操作。** 一个多余的 commit 没有任何代价，一次误删可能要重做半天。

---

## 1. 心智模型：三个区 + 一条历史线

```
   工作区                暂存区                 仓库（本地历史）
┌──────────┐   git add  ┌──────────┐ git commit ┌──────────┐
│ 磁盘上的  │ ─────────> │ 准备提交的 │ ──────────> │  提交记录  │
 │ 真实文件  │            │  快照     │             │  (commit) │
└──────────┘            └──────────┘             └──────────┘
     │                                                │
     └──────────── git restore <文件> ────────────────┘
                  （用仓库里的版本覆盖工作区）
```

- **工作区**：你在编辑器里改的文件。
- **暂存区**：`git add` 之后，git 知道"这些改动我下次要一起提交"。
- **仓库**：`git commit` 之后形成的不可变历史节点，每个节点有一个哈希（如 `2ea2642`）。

日常就是：改文件 → `add` → `commit`。看历史是往回看这条线，回退是把工作区/历史挪回线上的某个点。

---

## 2. 日常四步（占 90% 使用量）

```bash
git status              # 我看得最频繁的命令：现在改了哪些文件
git add -- configs/standard3.yaml  # 示例：只暂存本次确认要提交的文件
git commit -m "说明"     # 提交，生成一个历史节点
git log --oneline -5    # 确认提交成功，看最近 5 条
git push               # 上传当前分支的提交到私有 GitHub 仓库
```

`git status` 的输出这样读：

```
 M io/cboard.cpp              <- 左边一列空、右边 M：改过但还没 add
M  src/standard_readonly.cpp  <- 左边 M：已经 add 进暂存区了
?? tests/foo.py              <- 新文件，git 还不认识
```

**提交信息写什么**：一句话说清"为什么改"，不是"改了什么"（diff 本身已经说了改了什么）。本项目惯例是加日期/主题前缀，例如：

```bash
git commit -m "[9.23] 修复 qdu_link_monitor 重连后 att_age_ms 不重置"
```

> `git commit -am "说明"` 是 `add` + `commit` 的合并写法，但**只对已被 git 跟踪的文件有效**，新文件仍然要先 `git add`。

---

## 3. 看历史（"以往版本"怎么看）

```bash
git log --oneline -20                 # 最近 20 条，一行一条
git log --oneline --all --graph -20   # 带分支图，看分支怎么分的
git log --oneline -- io/cboard.cpp    # 只看某个文件的历史
git log -1 --stat                     # 上一条提交改了哪些文件、各加了多少行
git show 2ea2642                      # 看某条提交的完整改动
git show 2ea2642:io/cboard.cpp        # 看那个版本里某个文件的全文
git diff                              # 工作区 vs 暂存区（我还没 add 的改动）
git diff --cached                     # 暂存区 vs 上一条提交（我 add 了但还没 commit 的）
git diff HEAD                         # 工作区 vs 上一条提交（以上两者合起来）
git diff 2ea2642 HEAD -- src/         # 两个版本之间某个目录的差异
```

实用组合：

```bash
# "我上周改过这个文件吗？改了什么？"
git log --oneline -- since="1 week ago" -- io/qdu_shared_topic.cpp
git show <那条的哈希>
```

---

## 4. 回退（按危险程度从低到高）

### 4.1 我改乱了，想丢掉某个文件的改动 → `git restore`（安全）

```bash
git restore io/cboard.cpp        # 用仓库里的版本覆盖工作区，我的改动没了
git restore .                    # 丢掉全部未提交改动（危险，但只丢"没提交过的"）
```

`git restore` **只影响没提交的东西**。提交过的历史一根毛都不会动。

### 4.2 我 `add` 错了，想撤出暂存区 → `git restore --staged`（安全）

```bash
git restore --staged io/cboard.cpp   # 从暂存区撤出，文件内容不动
```

### 4.3 我想临时把当前改动收起来，先去干别的 → `git stash`（安全）

```bash
git stash                 # 把工作区+暂存区全部改动收起来，工作区变干净
git stash list            # 看收了哪些
git stash pop             # 取回最近一条（取回后从栈里删掉）
git stash drop            # 删掉最近一条
```

典型场景：正在改 detector，突然要验证一下"原始版本能不能编译过"。

```bash
git stash && cmake --build build -j4 ; git stash pop
```

### 4.4 我想撤销"上一条提交的内容"，但要保留更早的历史 → `git revert`（安全）

```bash
git revert HEAD           # 生成一条新提交，内容刚好抵消上一条
git revert 2ea2642        # 抵消指定的那条
```

**这是唯一适合"已经推给别人/已经同步到树莓派之后"的撤销方式**——它是加法，不改写历史。会弹编辑器让你写提交信息，`Ctrl+O` 回车保存 / `Ctrl+X` 取消（nano），或 `git revert --no-edit HEAD` 跳过。

### 4.5 我想回到过去的某个状态，历史也一起回退 → `git reset`（危险）

```bash
git reset --soft 2ea2642   # 历史回到那条，但我的改动全部保留在暂存区（可重新 commit）
git reset --mixed 2ea2642  # 默认。历史回去，改动保留在工作区（未 add）
git reset --hard 2ea2642   # 历史回去，改动全部丢弃 —— 会丢数据
```

三个选项的区别只在**"我那点没提交的改动怎么办"**：

| 选项 | 历史 | 未提交的改动 |
|---|---|---|
| `--soft` | 回退 | **保留**在暂存区 |
| `--mixed` | 回退 | **保留**在工作区 |
| `--hard` | 回退 | **丢弃** |

**`--hard` 不是世界末日**：它只丢"当时还没提交的东西"。提交过的内容 `git reset --hard HEAD@{1}` 或 `git reflog` 还能捞回来（见 §4.7）。但养成"先 commit 再 reset"的习惯最省事。

### 4.6 提交信息打错了 → `git commit --amend`（轻量，只改最新一条）

```bash
git commit --amend -m "改正后的说明"
```

只能改**最新一条**，且如果已经 push 过就不要用。

### 4.7 最后一道保险：`git reflog`（能救回"我以为丢了"的提交）

```bash
git reflog                # 本地 HEAD 的每一次移动，包括被 reset 掉的提交
git reset --hard HEAD@{3} # 回到 reflog 里那条状态
```

场景：`git reset --hard` 回退过头了，发现刚才那个 commit 里有要的东西。

```bash
git reflog               # 找到那条的哈希，例如 2ea2642
git reset --hard 2ea2642  # 回去
```

---

## 5. 分支（可选，但值得会用）

本项目目前是单线开发，`main` 上直接提交完全够用。当你要做一次**可能搞砸、想随时整体放弃**的改动时再开分支：

```bash
git switch -c 9.23-try-new-nms     # 从当前状态开一条新分支并切过去
# ... 随便改，随便提交 ...
git switch main                    # 切回主线（分支上的提交不影响 main）
git switch 9.23-try-new-nms        # 再切回去继续
git branch -D 9.23-try-new-nms     # 分支不要了，删掉（提交记录也一起删）
git stash && git switch main       # 有未提交改动时切分支会失败，先 stash
```

经验法则：**改动超过半天、且不确定要不要的时候，开分支**。其余直接提交到 `main`。

---

## 6. 本项目的特殊约定

### 6.1 `build/` 和素材不入库

`.gitignore` 决定了哪些文件 git 视而不见。当前排除了：

- `build/` `logs/` `records*`（原有）
- `assets/demo/`、`assets/*.onnx`、`io/mindvision/` —— 28 MB 的 mindvision SDK 和 60 MB 的 demo.avi 没有任何配置或代码引用；`io/CMakeLists.txt` 只按绝对路径链 hikrobot，且注释明确要求"生产 IO 目标只保留一个厂商后端"。
- `.vs/`、`**/.claude/settings.local.json` —— 本机 IDE/工具状态

**这些文件仍在磁盘上**，git 只是不跟踪。想改这个规则就编辑 `.gitignore`，然后：

```bash
git rm -r --cached <目录>    # 把已跟踪的某个目录撤出 git（不动磁盘文件）
```

### 6.2 为什么 `io/hikrobot/lib/` 那么大还是入库了

`io/CMakeLists.txt` 里：

```cmake
set(SP_HIKROBOT_LIBRARY ".../hikrobot/lib/arm64/libMvCameraControl.so")
foreach(... )
  if(NOT EXISTS "${SP_HIKROBOT_RUNTIME_FILE}")
    message(FATAL_ERROR "HikRobot runtime component not found: ...")
```

缺失直接 configure 失败，所以这 ~23 MB 厂商运行时是**构建依赖**，必须跟着仓库走。

### 6.3 换行符：`core.autocrlf=true`

Windows 主机、WSL、树莓派三处操作同一份代码。配置为：

- **仓库里统一存 LF**（Linux 编译需要）
- **Windows 工作区取出时是 CRLF**（Windows 编辑器习惯）

所以可能出现"我什么都没改，git 却说文件改了"——那是换行符被规范化了一次，`git add -A` 提交一次就好，不会反复出现。

### 6.4 中文路径正常显示

已设 `core.quotepath=false`，否则 `git status` 会把中文显示成 `\344\270\255` 这类转义。

### 6.5 电脑、WSL、GitHub 与树莓派的分工

Git 在电脑项目目录中管理。WSL 使用 `/mnt/d/workspace/sp_vision_25-main` 时访问的是同一电脑工作区，不是另一份 Pi 仓库。
GitHub 保存已推送的提交和分支，是远程副本；没有提交的新照片不会因 `push` 自动入库。
树莓派继续通过完整源码包解压和 `build/pi-sync/verify_source.py` 校验更新，不复制 `.git`，不生成源码备份。
只有 YAML 修改时，已有程序通常重启即可读取；C++ 修改后才按受影响目标重新编译。

---

## 7. 危险操作黑名单

| 命令 | 为什么危险 |
|---|---|
| `git reset --hard` | 丢弃所有未提交改动 |
| `git restore .` | 同上，只是范围小一点 |
| `git clean -fd` | **删除所有未被跟踪的文件**，包括你新建但还没 `git add` 的源码——本项目里也可能碰到被 `.gitignore` 排除的大文件。用之前先 `git clean -nd`（dry run）看看会删什么 |
| `git checkout <哈希> -- .` | 用旧版本覆盖整个工作区 |
| `git branch -D` | 删分支连提交记录一起删 |
| `git push --force` | 覆盖远端历史，会把别人的提交抹掉 |

**共同的安全习惯**：执行上面任何一条之前，先 `git status` + `git log --oneline -3`，必要时先 `git stash` 或 `git commit`。

---

## 8. 一分钟速查

```bash
# 今天干了什么 / 要提交什么
git status
git diff

# 存档一个节点
git add -- <本次修改的文件>
git commit -m "做了什么"
git push

# 回头看
git log --oneline -10
git show <哈希>
git show <哈希>:<文件路径>

# 改坏了，丢改动
git restore <文件>
git restore .

# 临时收起来
git stash / git stash pop

# 撤销已提交的一条（保留历史）
git revert <哈希>

# 回到过去（会动历史）
git reset --soft <哈希>    # 改动留着
git reset --hard <哈希>    # 改动丢了

# 我以为reset丢了东西
git reflog
```

---

## 9. 相关文档

- `9.22-qdu-cboard-simulator-readonly-chain.md` —— 本基线提交时所记录的工作内容
- `9.21-whole-project-status.md` —— 项目整体状态
