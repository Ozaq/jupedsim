# Usage guide

## Build Container

Use repo root as cwd and point to docker file with -f, e.g.:

```
docker build -t perf-test -f container/perf-measurement/Dockerfile .
```

The script orcestrating the perf test is baked into the container, i.e.
`container/perf-measurement/container_perf_test.py`. All other code needs to be
mounted into the container (see running container)

## Running Container

```
docker run \
    --cap-add CAP_PERFMON \
    --cap-add CAP_SYS_PTRACE \
    --ulimit memlock=-1 \
    --pid=host \
    -v<HOST-SOURCE-LOCATION>:/src \
    -v<HOST-BUILD-LOCATION>:/build \
    -it perf-test
```

The perf run will take several minutes and generate a `results` folder in the
supplied build path.

## Known Issues

Albeit supported, python function names do not show up in the flamegraphs. See
[Python Docs] (https://docs.python.org/3/howto/perf_profiling.html)

