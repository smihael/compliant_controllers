#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_ROOT=/home/user/controllers_ws
REPO_ROOT=/home/user/controllers_ws/src/compliant_controllers
BASHRC=/home/user/.bashrc

cd ${WORKSPACE_ROOT}
sudo chown -R user:user .

source /opt/ros/${ROS_DISTRO}/setup.bash
source /franka_ws/install/setup.bash
source /lbr_ws/install/setup.bash
if [ -f /controllers_ws/install/setup.bash ]; then
	source /controllers_ws/install/setup.bash
fi

rosdep update

mkdir -p src

git config --global --add safe.directory ${REPO_ROOT}
vcs import src < ${REPO_ROOT}/repos-compliant-controllers.yaml --skip-existing

rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y

colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

if ! grep -Fq 'source /home/user/controllers_ws/install/setup.bash' ${BASHRC}; then
	cat <<'EOF' >> ${BASHRC}
if [ -f /franka_ws/install/setup.bash ]; then source /franka_ws/install/setup.bash; fi
if [ -f /lbr_ws/install/setup.bash ]; then source /lbr_ws/install/setup.bash; fi
if [ -f /controllers_ws/install/setup.bash ]; then source /controllers_ws/install/setup.bash; fi
if [ -f /home/user/controllers_ws/install/setup.bash ]; then source /home/user/controllers_ws/install/setup.bash; fi
EOF
fi

echo "Dev container setup complete."
echo "Workspace: ${WORKSPACE_ROOT}"
echo "Build with: colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"