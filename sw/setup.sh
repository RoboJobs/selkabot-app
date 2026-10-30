# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install CUDA12.6
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-ubuntu2404.pin
sudo mv cuda-ubuntu2404.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.6.0/local_installers/cuda-repo-ubuntu2404-12-6-local_12.6.0-560.28.03-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2404-12-6-local_12.6.0-560.28.03-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2404-12-6-local/cuda-26DFA2FB-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-6

# Install cuDNN9.20.0
wget https://developer.download.nvidia.com/compute/cudnn/9.20.0/local_installers/cudnn-local-repo-ubuntu2404-9.20.0_1.0-1_amd64.deb
sudo dpkg -i cudnn-local-repo-ubuntu2404-9.20.0_1.0-1_amd64.deb
sudo cp /var/cudnn-local-repo-ubuntu2404-9.20.0/cudnn-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cudnn