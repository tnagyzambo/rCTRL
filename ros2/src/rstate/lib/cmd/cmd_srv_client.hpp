#include <cmd/cmd_srv.hpp>

namespace rstate {
    // Interface for templated CmdClient
    class CmdServiceClientIface {
    public:
        virtual ~CmdServiceClientIface() {}
    };

    // Class to manage construction of all Cmd's of a shared type
    template <typename T>
    class CmdServiceClient : public CmdServiceClientIface {
    public:
        CmdServiceClient(toml::table,
                         std::string,
                         std::shared_ptr<rclcpp::Client<T>>,
                         std::vector<const char *>,
                         std::map<uint, std::shared_ptr<CmdIface>> &);

    private:
        std::string serviceName;
        std::shared_ptr<rclcpp::Client<T>> client;
        std::vector<std::shared_ptr<CmdService<T>>> cmds;

        std::vector<std::shared_ptr<CmdService<T>>> createCmds(toml::table,
                                                               std::shared_ptr<rclcpp::Client<T>>,
                                                               std::vector<const char *>,
                                                               std::map<uint, std::shared_ptr<CmdIface>> &);
    };

    template <typename T>
    CmdServiceClient<T>::CmdServiceClient(toml::table toml,
                                          std::string serviceName,
                                          std::shared_ptr<rclcpp::Client<T>> client,
                                          std::vector<const char *> configSections,
                                          std::map<uint, std::shared_ptr<CmdIface>> &cmdMap) {
        this->serviceName = serviceName;
        this->client = client;
        this->cmds = createCmds(toml, this->client, configSections, cmdMap);
    }

    template <typename T>
    std::vector<std::shared_ptr<CmdService<T>>> CmdServiceClient<T>::createCmds(
        toml::table toml,
        std::shared_ptr<rclcpp::Client<T>> client,
        std::vector<const char *> configSections,
        std::map<uint, std::shared_ptr<CmdIface>> &cmdMap) {
        std::vector<std::shared_ptr<CmdService<T>>> cmds;
        for (auto section : configSections) {
            auto elems = util::getCmdsAsArray(*(toml[section]).as_table(), "cmd_service");
            if (elems != nullptr) {
                for (toml::node &elem : *elems) {
                    if (util::getServiceName(*elem.as_table()) == this->serviceName) {
                        uint id;
                        std::shared_ptr<CmdService<T>> cmd_ptr;
                        if (!strcmp(section, "shutdown_unconfigred") || !strcmp(section, "shutdown_inactive") ||
                            !strcmp(section, "shutdown_active") || !strcmp(section, "shutdown_armed")) {
                            auto cmd = CmdService(client, *elem.as_table());
                            id = cmd.id;
                            cmd_ptr = std::make_shared<CmdService<T>>(cmd);
                        } else {
                            auto cmd = CmdServiceCancelable(client, *elem.as_table());
                            id = cmd.id;
                            cmd_ptr = std::make_shared<CmdService<T>>(cmd);
                        }

                        // Register base interface of cmd in cmdMap with unique ID
                        auto insertResult = cmdMap.insert(std::pair<uint, std::shared_ptr<CmdIface>>(id, cmd_ptr));

                        // This should never throw, but just for safety
                        if (insertResult.second == false) {
                            std::stringstream error;

                            error << "ID collision during cmdMap insertion!\n";
                            error << "ID: " << id << "\n";
                            error << "Section: " << toml << "\n";

                            throw except::config_parse_error(error.str());
                        }

                        cmds.push_back(cmd_ptr);
                    }
                }
            }
        }

        return cmds;
    }
} // namespace rstate
