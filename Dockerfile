FROM ros:foxy

RUN apt update && apt install pylint -y

WORKDIR /root
CMD ["bash", "test-env.sh" ]

