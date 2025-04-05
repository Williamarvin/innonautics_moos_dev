# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Set the working directory inside the container
WORKDIR /app

# Install necessary dependencies for building and running the application
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install --assume-yes tzdata && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    apt-get install --assume-yes \
        build-essential \
        cmake \
        xterm \
        subversion \
        libfltk1.3-dev \
        libtiff5-dev \
        dos2unix \
        libopencv-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy all project files into the container
COPY . .

# Ensure scripts have Unix-style line endings
RUN find /app -type f -name "*.sh" -exec dos2unix {} +

# Set the environment variable for the application's port
ENV PORT=8080

# Expose the port for external access
EXPOSE 8080

# Ensure build scripts are executable
RUN chmod +x /app/moos-ivp/build.sh && \
    chmod +x /app/moos-ivp-pavlab-aro/build.sh && \
    chmod +x /app/moos-ivp-pavlab-aro/missions/alpha_heron/*.sh

RUN /bin/bash -c "./moos-ivp/build.sh"
RUN /bin/bash -c "./moos-ivp-pavlab-aro/build.sh"

# Create useful aliases for convenience
RUN echo 'alias cdr="moos-ivp-pavlab-aro"' >> /root/.bashrc && \
    echo 'alias cda="moos-ivp-pavlab-aro/missions/alpha_heron"' >> /root/.bashrc

# Add the binary directories to the PATH environment variable
ENV PATH="/app/moos-ivp/ivp/bin:/app/moos-ivp-pavlab-aro/bin:$PATH"

RUN bash -c "source /root/.bashrc"

# Copy and fix entrypoint script
COPY entrypoint.sh /app/entrypoint.sh
RUN dos2unix /app/entrypoint.sh && chmod +x /app/entrypoint.sh

# Set the entrypoint script
ENTRYPOINT ["/app/entrypoint.sh"]
