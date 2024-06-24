FROM makaspacex/smartwchr:dev

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

CMD ["zsh"]
