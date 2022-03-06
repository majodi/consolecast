/* HTTP GET handlers (served files are embedded in the firmware)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "cc_http.h"


esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    extern const char index_html_start[] asm("_binary_index_html_start");
    extern const char index_html_end[]   asm("_binary_index_html_end");
    httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

esp_err_t favicon_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/x-icon");
    extern const char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    httpd_resp_send(req, favicon_ico_start, favicon_ico_end - favicon_ico_start);
    return ESP_OK;
}

esp_err_t ccpng_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/png");
    extern const char ccpng_start[] asm("_binary_cc_png_start");
    extern const char ccpng_end[]   asm("_binary_cc_png_end");
    httpd_resp_send(req, ccpng_start, ccpng_end - ccpng_start);
    return ESP_OK;
}

esp_err_t xterm_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript; charset=UTF-8");
    extern const char xterm_js_start[] asm("_binary_xterm_js_start");
    extern const char xterm_js_end[]   asm("_binary_xterm_js_end");
    httpd_resp_send(req, xterm_js_start, xterm_js_end - xterm_js_start);
    return ESP_OK;
}
