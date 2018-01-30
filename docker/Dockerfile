FROM udacity/controls_kit:latest

RUN apt-get update && \
	apt-get install -y \
        build-essential \
        curl \
        gdb \
        gdbserver \
        git \
        openssh-server \
        sudo && \
    apt-get clean

RUN mkdir /var/run/sshd && \
	echo 'root:toor' | chpasswd && \
	sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
	sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

RUN mkdir -p /opt/cmake-3.9.0 && \
        curl -sSL https://cmake.org/files/v3.9/cmake-3.9.0-Linux-x86_64.tar.gz | tar -xzC /opt/cmake-3.9.0 --strip-components=1

VOLUME /usr/src

WORKDIR /usr/src

EXPOSE 22

CMD ["/usr/sbin/sshd", "-D"]
