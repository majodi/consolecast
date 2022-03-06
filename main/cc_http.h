/* HTTP GET handlers (served files are embedded in the firmware)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once

#include "cc_webserver.h"


esp_err_t root_get_handler(httpd_req_t *req);
esp_err_t favicon_get_handler(httpd_req_t *req);
esp_err_t ccpng_get_handler(httpd_req_t *req);
esp_err_t xterm_get_handler(httpd_req_t *req);
