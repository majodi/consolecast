idf_component_register(SRCS "main.c"
                            "cc_wifi.c"
                            "cc_webserver.c"
                            "cc_http.c"
                            "cc_websocket.c"
                            "cc_indicator.c"
                            "cc_uart.c"
                            "cc_cdc_acm.c"
                            "keep_alive.c"
                           
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "certs/servercert.pem"
                                   "certs/prvtkey.pem"
                                   "html/index.html"
                                   "html/favicon.ico"
                                   "html/cc.png"
                                   "html/xterm.js"
                )

