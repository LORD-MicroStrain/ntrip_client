.PHONY: all build build-shell clean

# These variables should be overridden in CI builds to configure this build
version ?= latest
docker ?= docker
arch ?= amd64
ros_version ?= noetic

# Just set some directory values to support out of tree builds
makefile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
docker_dir := $(dir $(makefile_path))
project_dir := $(abspath $(docker_dir)/..)
project_name := $(shell echo $(notdir $(project_dir)) | tr A-Z a-z)
artifacts_dir := $(docker_dir)/.artifacts/$(arch)-$(ros_version)
build_dir := $(artifacts_dir)/build
devel_dir := $(artifacts_dir)/devel
docker_catkin_root := /home/microstrain/catkin_ws
docker_catkin_src_dir := $(docker_catkin_root)/src
docker_catkin_build_dir := $(docker_catkin_root)/build
docker_catkin_devel_dir := $(docker_catkin_root)/devel

# All of these directories have to exist before we can run most tasks
dir_deps := $(artifacts_dir) $(build_dir) $(devel_dir)

# Grab the user and group ID so that the files we create can be deleted and managed from the base system
user_id := $(shell id -u)
group_id := $(shell id -g)

# Set up some variables for the docker build
dev_dockerfile := $(docker_dir)/Dockerfile.dev
build_args := --build-arg http_proxy --build-arg https_proxy --build-arg no_proxy --build-arg ARCH=$(arch) --build-arg ROS_VERSION=$(ros_version) --build-arg USER_ID=$(user_id) --build-arg GROUP_ID=$(group_id)
run_args := -e http_proxy -e https_proxy -e no_proxy
run_mounts := -v "$(project_dir):$(docker_catkin_src_dir)/$(project_name)" -v "$(build_dir):$(docker_catkin_build_dir)" -v "$(devel_dir):$(docker_catkin_devel_dir)"
dev_image_name := $(arch)/$(project_name)-dev:$(ros_version)
dev_image_artifact := $(artifacts_dir)/.image

all: image

$(dev_image_artifact): $(dev_dockerfile) | $(dir_deps)
	@$(docker) build \
		-t $(dev_image_name) \
		-f $(dev_dockerfile) \
		$(build_args) \
		$(project_dir)
	@echo $(dev_image_name) > $@

build-shell: $(dev_image_artifact)
	@$(docker) run \
		-it \
		--rm \
		--privileged \
		--net="host" \
		--user "microstrain" \
		-w $(docker_catkin_root) \
		--entrypoint="/bin/bash" \
		$(run_args) \
		$(run_mounts) \
		$$(cat $<) -c " \
			sudo apt-get update; \
			rosdep install --from-paths $(docker_catkin_src_dir) --ignore-src -r -y; \
			/bin/bash; \
		"

$(artifacts_dir):
	@mkdir -p $@

$(build_dir):
	@mkdir -p $@

$(devel_dir):
	@mkdir -p $@

clean:
	@rm -rf "$(build_dir)" "$(devel_dir)"
	@rm -f "$(dev_image_artifact)"
	@docker ps -a | grep "$(project_name)" | grep "$(arch)" | grep "$(ros_version)" | tr -s " " | cut -d' ' -f1 | xargs docker rm -f || echo "No containers to remove"
	@docker images | grep "$(project_name)" | grep "$(arch)" | grep "$(ros_version)" | tr -s " " | cut -d' ' -f1 | xargs docker rmi -f || echo "No images to remove"
