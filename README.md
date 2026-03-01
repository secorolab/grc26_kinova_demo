# grc26 kinova demo

## Task

### realworld

- A single kinova gen3 arm mounted on a table top.
- A tray setup on the table.
- The arm moves compliantly down till it made contact with the table.
- After contact, it slides forward to make contact with the tray (fixed position)
- It then waits for a human to hold the tray on the other side.
- Once human holds, the arm initiates movement upwards to place tray in a diff. place.
- The arm reacts compliantly to the human while in motion.
- Task completes, once the tray is placed in the target position on the table.

### simulation - isaacscim 5.1.0

- another kinova robot acts as an agent instead of human
- no compliance behaviour
- uses moveit for motion planning


## Setup

### clone the repo

```shell
mkdir -p ~/grc26/src

cd ~/grc26/src

git clone --depth=1 https://github.com/secorolab/grc26_kinova_demo.git
```

### pull the dependent repos

```shell
vcs import src < src/grc26_kinova_demo/grc26.repos
```

