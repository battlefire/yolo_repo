#include "TransferJson.h"

string globle_str;
bool switch_seq = false;

void json_obj_box::datapack_vehicle_json_packge_and_updata()
{
	road_data_to_server temp_Vdata;
    char detail_name[INDEX_NAME_LENTH] = "module_ooj_idx";
    char temp_detail_name[INDEX_NAME_LENTH];
    //json_object *vehicle_data_brige = NULL;
    //json_object *detail_obj         = NULL;
    this->vehicle_jsonobject_list   = NULL;
    json_object*vehicle_data_brige_list[MAX_OBJ_NUM];
    //vehicle_data_brige            = json_object_new_object();
    //detail_obj                    = json_object_new_object();
	this->vehicle_jsonobject_list = json_object_new_object();

    for(int vehicle_data_looptime = 0; vehicle_data_looptime < this->data_vehicle_num; vehicle_data_looptime++)
    {
        vehicle_data_brige_list[vehicle_data_looptime]=json_object_new_object();
        sprintf(temp_detail_name,"%s%d",detail_name,vehicle_data_looptime);
        memcpy(&temp_Vdata,&data_obj_Vdata[vehicle_data_looptime],sizeof(temp_Vdata));
        json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime],"frame_num", json_object_new_int(temp_Vdata.frame_num));
        json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime],"module_id",json_object_new_int(temp_Vdata.modul_id));
        json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime],"module_type",json_object_new_int(temp_Vdata.type));
        json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime],"module_location_x",json_object_new_double(temp_Vdata.x_loca));
        json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime],"module_location_y",json_object_new_double(temp_Vdata.y_loca));
        json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime],"module_high",json_object_new_double(temp_Vdata.high));
        json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime],"module_wide",json_object_new_double(temp_Vdata.wide));
        json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime],"module_speed",json_object_new_double(temp_Vdata.speed));
        json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime],"module_direction",json_object_new_double(temp_Vdata.direction));
		json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime], "camera_longitude", json_object_new_double(temp_Vdata.longitude));
		json_object_object_add(vehicle_data_brige_list[vehicle_data_looptime], "camera_latitude", json_object_new_double(temp_Vdata.latitude));
        json_object_object_add(vehicle_jsonobject_list,temp_detail_name,vehicle_data_brige_list[vehicle_data_looptime]);
        bzero(&temp_Vdata,sizeof(temp_Vdata));
       };
	json_send_str = (char *)json_object_to_json_string(vehicle_jsonobject_list);
    transfer_str = (char*)malloc(json_send_str.length()+HEAD_LENTH_4G + 1);
    transfer_str[0] = (char)data_vehicle_num;
    transfer_str[1]=json_send_str.length()/128;
    transfer_str[2] = json_send_str.length() - transfer_str[1]*128;
	memcpy(&transfer_str[3],json_send_str.c_str(),json_send_str.length());
    transfer_str[json_send_str.length()+1] = '\0';
	json_send_str = transfer_str;
    int len_json_str_echo = transfer_str[1]*128 +transfer_str[2];
    while(switch_seq == false)
    {
        updata_4G();
        switch_seq  = true;
    }
    printf("------------------len:%d\n",len_json_str_echo);
    free(transfer_str);
    return;
}

void json_obj_box::updata_4G()
{
    globle_str = this->json_send_str;
    empty_data_vector();
}

void json_obj_box::set_data_vehicle_and_datavector(int vehicle_num,vector<road_data_to_server> &Vdata)
{
        int temp_vehicle = 0;
        this->data_vehicle_num = vehicle_num;
        for(;temp_vehicle<vehicle_num;temp_vehicle++)
        {
            this->data_obj_Vdata.push_back(Vdata[temp_vehicle]);
        }
        datapack_vehicle_json_packge_and_updata();
}

void json_obj_box::empty_data_vector()
{
    data_vehicle_num = 0;
    data_obj_Vdata.clear();
}


