FROM gcc:4.9

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get -qy update && apt-get -qy dist-upgrade && \
    apt-get -qy --no-install-recommends install \
    bc \
    lib32stdc++6 \
    lib32z1 \
    libc6-dev-i386 \
    libc6-i386 \
    ;
