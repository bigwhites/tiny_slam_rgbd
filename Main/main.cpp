#include <common_include.h>
#include <DS.h>
#include <Vo.h>

// 主函数
int main(int argc, char **argv)
{
    // freopen("./log.log", "w", stdout);
    string configFileList = "../../config.yaml";
    if (argc > 2)
    {
        configFileList = argv[1];
    }

    auto vo = make_unique<Vo>();
    vo->init(configFileList);
    vo->applictionStart();
    return 0;
}