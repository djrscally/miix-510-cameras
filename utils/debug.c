#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/media.h>

int open_device(char *devname)
{
    int ret, fd;

    fd = open(devname, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open the device: %s\n", strerror(errno));
        return 1;
    }

    return fd;
}

int read_device_info(int fd)
{
    int ret;
    struct media_device_info mdi = {};

    ret = ioctl(fd, MEDIA_IOC_DEVICE_INFO, &mdi);

    if (ret < 0) {
        fprintf(stderr, "Failed to read device info: %s\n", strerror(errno));
        return -1;
    }

    fprintf(stdout, "Media device info\ndriver: %s, model: %s, serial: %s, bus_info: %s\n", mdi.driver, mdi.model, mdi.serial, mdi.bus_info);

    return 0;
}

int enumerate_topology(int fd)
{
    int ret;
    struct media_v2_topology top = {};

    ret = ioctl(fd, MEDIA_IOC_G_TOPOLOGY, &top);

    if (ret < 0) {
        fprintf(stderr, "Failed to read device topology info: %s\n", strerror(errno));
        return -1;
    }

    fprintf(stdout, "\nListing entities\nnum entities: %d, num interfaces: %d, num pads: %d, num links: %d\n",
                top.num_entities, top.num_interfaces, top.num_pads, top.num_links);

    struct media_v2_entity entities[top.num_entities];

    top.ptr_entities = (unsigned long)&entities;

    ret = ioctl(fd, MEDIA_IOC_G_TOPOLOGY, &top);

    if (ret < 0) {
        fprintf(stderr, "Failed to read device topology info: %s\n", strerror(errno));
        return -1;
    }

    for (int i = 0; i < top.num_entities; i++) {
        fprintf(stdout, "Entity found: %s (%d)\n", entities[i].name, entities[i].id);
    }

    return 0;
}

int main(int argc, char *argv[])
{
    int ret, fd;

    /* open media device */
    fd = open_device(argv[1]);

    /* read and print the dev info */
    ret = read_device_info(fd);

    if (ret < 0) {
        close(fd);
        return -1;
    }

    ret = enumerate_topology(fd);

    if (ret < 0) {
        close(fd);
        return -1;
    }

    /* need to get num pads for entity 1 */
    struct media_entity_desc entity = {
        .id=atoi(argv[2])
    };

    ret = ioctl(fd, MEDIA_IOC_ENUM_ENTITIES, &entity);
    if (ret < 0) {
        fprintf(stderr, "Failed to get Entity info: %d\n", ret);
    }

    fprintf(stdout, "\nChecking entity %s: num_pads: %d, num outbound links: %d\n", entity.name, entity.pads, entity.links);

    struct media_pad_desc pads[entity.pads];
    struct media_link_desc links[entity.links];

    struct media_links_enum lenum = {
        .entity=atoi(argv[2]),
        .pads=pads,
        .links=links,        
    };

    ioctl(fd, MEDIA_IOC_ENUM_LINKS, &lenum);
    if (ret < 0) {
        fprintf(stderr, "Failed to get links info: %d\n", ret);
    }

    for (int i = 0; i < entity.links; i++) {
        fprintf(stdout, "    source id %d[%d] -> sink id %d[%d]\n",
                        lenum.links[i].source.entity,
                        lenum.links[i].source.index, 
                        lenum.links[i].sink.entity,
                        lenum.links[i].sink.index
        );
    }


    return 0;
}