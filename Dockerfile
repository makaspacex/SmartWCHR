FROM makaspacex/smartwchr:dev-latest

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

CMD ["bash"]
