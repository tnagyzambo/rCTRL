#include <util/util.hpp>

namespace rstate::util {
    // Helper function for getTomlTableBySections()
    // Change input point argument 'toml' to the table found with the section key
    // Throw if no table exists
    void pointTomlToTableBySection(toml::table *toml, const char *section) {
        toml::node_view view = (*toml)[section];

        if (view.type() != toml::node_type::table) {
            std::stringstream error;

            error << "Unable to parse toml section!\n";
            error << "Section: " << section << "\n";
            error << "TOML: " << view << "\n";

            throw rstate::except::config_parse_error(error.str());
        }

        toml = view.as_table();
    }

    // Create the ROS2 service name that the client is targeting from the node name and the service name
    std::string getServiceName(toml::table toml) {
        std::string serviceName = getTomlEntryByKey<std::string>(toml, "node");
        serviceName.append("/");
        serviceName.append(getTomlEntryByKey<std::string>(toml, "service"));

        return serviceName;
    }

    // Helper function to check if a table contains a cmd before casting to to an array
    // Attemping to call .as_array() on an empty table results in an execption
    // I have not found a nice way to to exception handling with TOML++
    // Instead we check and return a nullptr to check against in the calling function
    toml::array *getCmdsAsArray(toml::table &toml, const char *cmd) {
        if (toml.contains(cmd)) {
            return toml[cmd].as_array();
        }
        return nullptr;
    }
} // namespace rstate::util
