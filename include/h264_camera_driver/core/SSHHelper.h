#ifndef SSH_HELPER_H
#define SSH_HELPER_H

#include <string>
#include <libssh/libssh.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

class SSHHelper {
public:
    SSHHelper(const std::string& hostname, const std::string& username, const std::string& password);
    ~SSHHelper();

    std::string executeCommand(const std::string& command);
    std::string getId(const std::string& remoteFilePath, const std::string& fieldName);

private:
    struct ssh_session_struct* session;
    bool connect();
    static const int RETRY_DELAY_MS = 2000;
};

#endif // SSH_HELPER_H
