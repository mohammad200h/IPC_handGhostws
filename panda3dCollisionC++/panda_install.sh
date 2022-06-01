sudo ./panda_dependencies.sh 
cd panda3d && python3 makepanda/makepanda.py --everything --installer --no-egl --no-gles --no-gles2 --no-opencv --no-bullet
sudo dpkg -i panda3d1.11_1.11.0_amd64.deb