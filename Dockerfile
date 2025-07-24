FROM ubuntu:22.04

RUN apt-get update && apt-get install -y git

RUN git clone https://github.com/onurcalisir27/LearningAdaptive.git /opt/LearningAdaptive

RUN chmod +x /opt/LearningAdaptive/scripts/ros_install_nosudo.sh

RUN bash /opt/LearningAdaptive/scripts/ros_install_nosudo.sh
RUN bash /opt/LearningAdaptive/scripts/pigpio_install.sh

CMD ["/bin/bash"]
