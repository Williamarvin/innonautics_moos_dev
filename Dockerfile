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
        libcurl4-openssl-dev \
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

# Run build scripts
RUN rm -rf /app/moos-ivp/build/MOOS/MOOSCore/CMakeCache.txt
RUN rm -rf /app/moos-ivp/build/MOOS/MOOSEssentials/CMakeCache.txt
RUN rm -rf /app/moos-ivp/build/MOOS/MOOSGeodesy/CMakeCache.txt
RUN rm -rf /app/moos-ivp/build/MOOS/MOOSMatlab/CMakeCache.txt
RUN rm -rf /app/moos-ivp/build/MOOS/MOOSToolsUI/CMakeCache.txt
RUN rm -rf /app/moos-ivp/build/ivp/CMakeCache.txt
RUN touch MOOS

RUN cd /app/moos-ivp && ./build.sh
RUN cd /app/moos-ivp-pavlab-aro && ./build.sh

# Add binary directories to the PATH permanently
ENV PATH="/app/moos-ivp/bin:/app/moos-ivp-pavlab-aro/bin:$PATH"

# Create useful aliases for convenience (add them to .bashrc)
RUN echo 'alias cdr="cd /app/moos-ivp-pavlab-aro"' >> /root/.bashrc && \
    echo 'alias cda="cd /app/moos-ivp-pavlab-aro/missions/alpha_heron"' >> /root/.bashrc

# Copy and fix entrypoint script
COPY entrypoint.sh /app/entrypoint.sh
RUN dos2unix /app/entrypoint.sh && chmod +x /app/entrypoint.sh

# Set the entrypoint script
ENTRYPOINT ["/app/entrypoint.sh"]