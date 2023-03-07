# **_Streamer_**

---

### Platforms

* Linux (Ubuntu20.04, x64, gcc-9.4.0)

---

### Dependencies

* CMake >3.8 (local=3.8)
* Qt >5.15.x  (local=3.15.2)
* Opencv >3.4.x  (local=3.4.15)
* VTK >7.1.x  (local=8.2.0, build qVtkWidget plugins)
* PCL >1.8.x  (local=1.9.1)

---

### Building

* git clone git@github.com:wangrichaoren/streamer.git
* cd streamer
* mkdir build && cd build
* cmake ..
* make -j12 && make install

---

### Case

![imshow.png](doc%2Fimshow.png)