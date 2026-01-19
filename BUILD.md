- run in the root directory
  ```shell
    rm -rf build

    rm -rf debian/eimu-serial*

    rm -rf build debian/eimu-serial* debian/.debhelper* debian/debhelper* debian/eimu-serial* debian/files obj-*
  ```

  ```shell
  sudo apt update

  sudo apt remove eimu-serial-dev

  sudo apt install \
    build-essential \
    cmake \
    dpkg-dev \
    debhelper \
    pkg-config \
    libserial-dev
  ```

  ```shell
  cmake -S . -B build

  cmake --build build
  ```

  ```shell
  dpkg-buildpackage -us -uc
  ```

- install built .deb package
  ```shell
    sudo apt remove libserial-dev #uninstall
    sudo apt install ../eimu-serial-dev_<version>_amd64.deb #this should install the libserialdev along
  ```

- check if installed
  ```shell
    dpkg -L eimu-serial-dev
  ```

- remove
  ```shell
    rm -rf build debian/eimu-serial* debian/.debhelper* debian/debhelper* debian/eimu-serial* debian/files obj-*
  ```
