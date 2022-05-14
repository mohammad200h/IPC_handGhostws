

echo "installing wget...."
echo "Y"| sudo apt-get install wget
echo "downloading large files from dropbox...."
echo "downloading Bullet3...."
wget https://www.dropbox.com/s/um0y03ee5tjyiri/bullet3.zip?dl=0
echo "downloading openVR...."
wget https://www.dropbox.com/s/hcikzwq3jskda94/openvr.zip?dl=0
echo "downloading LinuxCommunication...."
wget https://www.dropbox.com/s/76gyoyldc3rfoqu/LinuxCommunication-main.zip?dl=0

echo "renaming downloaded files...."

mv ./bullet3.zip?dl=0                   bullet3.zip
mv ./openvr.zip?dl=0                    openvr.zip
mv ./LinuxCommunication-main.zip?dl=0   LinuxCommunication.zip

echo "unziping downloaded files..."

echo "y" | unzip bullet3.zip
echo "y" | unzip openvr.zip
echo "y" | unzip LinuxCommunication.zip

echo "Installing packages..."



echo "installing openvr..."
cd openvr/ 
rm -r build 
mkdir build 
cd build 
cmake .. 
make 
sudo make install

cd ../..

echo "installing bullet3..."
cd bullet3/
rm -r build 
mkdir build 
cd build 
cmake .. 
make 
sudo make install

cd ../..

echo "installing LinuxCommunication..."
cd LinuxCommunication-main/GymGhostWordCom/
rm -r build 
mkdir build 
cd build 
cmake .. 
make 
sudo make install
cd ..
sudo pip3 install -e .


echo "End of Installation :D"
