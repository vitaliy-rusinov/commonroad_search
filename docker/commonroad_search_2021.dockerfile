FROM continuumio/anaconda3

RUN conda create -n commonroad-py37 python=3.7
# add command to bashrc
RUN echo "source activate commonroad-py37" > ~/.bashrc
# # link python3 to python3.7
# RUN rm /usr/bin/python3 && ln -sf python3.7 /usr/bin/python3
# install packages
RUN apt-get update && apt-get install \
    imagemagick -y \
    python3-dev -y \
    make \
    build-essential \
    m4 \
    libboost-dev \
    libboost-thread-dev \
    libboost-test-dev \
    libboost-filesystem-dev \
    cmake \
    libeigen3-dev \ 
    socat \
    tk-dev -y
RUN bash -ic 'pip install jupyter tqdm imageio pyyaml ipywidgets networkx'

# create and switch to commonroad directory
WORKDIR /commonroad

# install commonroad-io
RUN pip install commonroad-io==2021.3

# install requirements of commonroad-search
RUN git clone https://gitlab.lrz.de/tum-cps/commonroad-search.git && \
    cd commonroad-search && pip install -r requirements.txt
# switch back to commonroad directory
WORKDIR /commonroad

# install commonroad-route-planner
RUN git clone https://gitlab.lrz.de/tum-cps/commonroad-route-planner.git && \
    cd commonroad-route-planner && pip install .
# switch back to commonroad directory
WORKDIR /commonroad

# build and install commonroad-drivability-checker
RUN git clone https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker.git && \
    cd commonroad-drivability-checker && pip install -r requirements.txt && \
    bash build.sh -e "/opt/conda/envs/commonroad-py37" -v 3.7 -i -j 4
# set environment variable
ENV PYTHONPATH /commonroad-drivability-checker/
# switch back to commonroad directory
WORKDIR /commonroad

# set entrypoint for docker image
# == this is the safer option
# ENTRYPOINT bash -c "\
# conda activate commonroad-py37 &&\
# cd /commonroad-search/notebooks;\
# socat TCP-LISTEN:8888,fork TCP:127.0.0.1:9000 &\
# jupyter notebook --ip 0.0.0.0 --no-browser --allow-root --port 9000 "

# == this is the more convenient option
ENTRYPOINT bash -c "source activate commonroad-py37 &&\
    jupyter notebook --ip 0.0.0.0 --no-browser --allow-root --NotebookApp.token='' --NotebookApp.password=''"

