#!/bin/bash
# Export CI environment info
export CODECOV_TOKEN=5de39e6d-c202-42e7-84b1-961bd00650e1
ci_env=$(bash <(curl -s https://codecov.io/env))
export DOCKER_RUN_OPTS="$DOCKER_RUN_OPTS $ci_env"
