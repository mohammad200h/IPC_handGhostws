  #include <pandaFramework.h>
   #include <pandaSystem.h>
   #include <animControlCollection.h>
   #include <auto_bind.h>
   #include <string>




   PandaFramework framework;

   using namespace std;

   int main(int argc, char* argv[])
   {
        string path = "/home/mamad/IPC_handGhostws/panda3dCollisionC++/src/";
        framework.open_framework(argc, argv);
        WindowFramework* win = framework.open_window();
        NodePath camera = win->get_camera_group();
        // NodePath teapot = win->load_model(framework.get_models(),"teapot");
        NodePath teapot = win->load_model(framework.get_models(),path+"FF.egg");
        teapot.reparent_to(win->get_render());
        teapot.set_pos(-5, 0, 0);
        NodePath panda = win->load_model(framework.get_models(),"panda");
        panda.reparent_to(win->get_render());
        panda.set_pos(5, 0, 0);
        win->load_model(panda, "panda-walk");
        AnimControlCollection pandaAnims;
        auto_bind(panda.node(), pandaAnims);
        pandaAnims.loop("panda_soft", false);
        camera.set_pos(0, -30, 6);
        framework.main_loop();
        framework.close_framework();
        return 0;
}