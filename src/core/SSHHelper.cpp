#include <h264_camera_driver/core/SSHHelper.h>

SSHHelper::SSHHelper(const std::string& hostname, const std::string& username, const std::string& password) {
    int attempts = 0;
    bool connected = false;

    while (!connected) {  // Infinite until successful connection
        attempts++;
        session = ssh_new();
        if (!session) {
            std::cerr << "Failed to create SSH session, attempt " << attempts << ". Retrying..." << std::endl;
            usleep(RETRY_DELAY_MS * 1000);
            continue;
        }

        ssh_options_set(session, SSH_OPTIONS_HOST, hostname.c_str());
        ssh_options_set(session, SSH_OPTIONS_USER, username.c_str());

        if (ssh_connect(session) != SSH_OK) {
            std::cerr << "SSH connection failed: " << ssh_get_error(session)
                      << ", attempt " << attempts << ". Retrying..." << std::endl;
            ssh_free(session);
            usleep(RETRY_DELAY_MS * 1000);
            continue;
        }

        if (ssh_userauth_password(session, nullptr, password.c_str()) != SSH_AUTH_SUCCESS) {
            std::cerr << "SSH authentication failed: " << ssh_get_error(session)
                      << ", attempt " << attempts << ". Retrying..." << std::endl;
            ssh_disconnect(session);
            ssh_free(session);
            usleep(RETRY_DELAY_MS * 1000);
            continue;
        }

        connected = true;
        std::cout << "SSH connection established successfully after " << attempts << " attempts" << std::endl;
    }
}

SSHHelper::~SSHHelper() {
    if (session) {
        ssh_disconnect(session);
        ssh_free(session);
    }
}

std::string SSHHelper::executeCommand(const std::string& command) {
    ssh_channel channel = ssh_channel_new(session);
    if (!channel || ssh_channel_open_session(channel) != SSH_OK) {
        throw std::runtime_error("Failed to open SSH channel.");
    }

    if (ssh_channel_request_exec(channel, command.c_str()) != SSH_OK) {
        ssh_channel_close(channel);
        ssh_channel_free(channel);
        throw std::runtime_error("Failed to execute command.");
    }

    std::stringstream output;
    char buffer[256];
    int nbytes;
    while ((nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0)) > 0) {
        output.write(buffer, nbytes);
    }

    ssh_channel_close(channel);
    ssh_channel_free(channel);

    return output.str();
}

std::string SSHHelper::getId(const std::string& remoteFilePath, const std::string& fieldName) {
    std::string command = "cat " + remoteFilePath;
    std::string fileContent = executeCommand(command);

    std::stringstream fileStream(fileContent);
    YAML::Node root = YAML::Load(fileStream);

    if (root[fieldName]) {
        return root[fieldName].as<std::string>();
    } else {
        throw std::runtime_error("Respective field not found in the YAML file.");
    }
}