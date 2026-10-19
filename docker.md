[Docker in 100 Seconds - YouTube](https://youtu.be/Gjnup-PuquQ?si=2Rk1wVbjfvZsggPg)
[The Only Docker Tutorial You Need To Get Started - YouTube](https://youtu.be/DQdB7wFEygo?si=7YYfqg8zUwM3I3IM)
[Docker in 4 minutes (No BS, No Fluff) - YouTube](https://youtu.be/YEyl9CS5oNY?si=IVZhgkRnGt_N1gcp)

- it can contain the code, dependencies, environment, etc.
	no matter who is using it and where, it will run

## images

ingredients and instructions
	technologies required, runtimes, tools, instructions
an image can create multiple container instances
acts as a blueprint / template for the container
	contains the application code, libraries, system tools, and dependencies
created through **dockerfiles**

generate image
``` bash
docker build .
```
generate container
``` bash
docker run "..."
```
## containers

the meal itself
	this actually runs the code
**docker compose** allows you to link up multiple different containers (described via .yaml file)
works via...
``` bash
docker compose up
```

## example

this circumvents the need to install python

### files

*example.py*
``` python
from time import sleep
from random import randint

while (True):
	print("example for docker")
	sleep(randint(0, 4))
```

*Dockerfile*
``` dockerfile
# Start from an official Python image
FROM python:3.12-slim

# Set the working directory inside the container
WORKDIR /app

# Copy your script into the container
COPY example.py .

# Command to run when the container starts
CMD ["python", "example.py"]
```

### process

build image
``` bash
docker build -t docker-example .
```

run generated container
``` bash
docker run --name my-example docker-example
```