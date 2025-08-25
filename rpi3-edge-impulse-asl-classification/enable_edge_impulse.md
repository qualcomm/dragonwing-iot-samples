# Connect Rubik Pi to Edge Impulse
This guide walks you through connecting a **Rubik Pi** (Debian/Ubuntu) to **Edge Impulse** for data collection and on-device testing. 

## 1) Prep the Rubik Pi
Update packages and install common dependencies:

```bash
sudo apt update
sudo apt install repo gawk wget git diffstat unzip texinfo gcc build-essential chrpath socat cpio python3 python3-pip python3-pexpect xz-utils debianutils iputils-ping python3-git python3-jinja2 libsdl1.2-dev pylint xterm python3-subunit mesa-common-dev zstd liblz4-tool locales tar python-is-python3 file libxml-opml-simplegen-perl vim whiptail bc
sudo apt-get install checkinstall libreadline-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev libffi-dev curl git-lfs libncurses5-dev libncursesw5-dev

python --version
# Download it in a directory of your choice
wget https://www.python.org/ftp/python/3.10.2/Python-3.10.2.tgz
tar -xvf Python-3.10.2.tgz
cd Python-3.10.2
./configure --enable-optimizations
make
sudo make install
sudo pip3.10 install pefile
```
## 2) Download Edge Impulse Linux CLI
``` bash
wget https://cdn.edgeimpulse.com/firmware/linux/setup-edge-impulse-qc-linux.sh
sh setup-edge-impulse-qc-linux.sh
source ~/.profile
```

## 3) Connect to Edge Impulse
``` bash
edge-impulse-linux
```
You will be prompted for your username, email, and password you used to set up your Edge Impulse account, so be sure to have those details ready. If you want to reset the setup or swtich devices, use:

``` bash
edge-impulse-linux --clean
```

## 3) Run your Impulse
``` bash
edge-impulse-linux-runner
```
