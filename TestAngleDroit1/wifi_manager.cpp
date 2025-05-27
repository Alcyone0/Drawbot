#include "wifi_manager.h"
#include <WiFi.h>
#include "RobotStructures.h"
#include "Sequences.h"

const char* ssid = "Drawbot_WIFI";
const char* password = "12345678";
WiFiServer server(80);
const char* COMPILE_DATE = __DATE__;
const char* COMPILE_TIME = __TIME__;

void setupWiFi() {
    WiFi.softAP(ssid, password);
    server.begin();
    String ipAddress = WiFi.softAPIP().toString();
    addLog("[setup] WiFi AP démarré - IP: " + ipAddress);
}

void handleWiFiClient() {
    if (deplacementFait) {
        WiFiClient client = server.available();
        if (client) {
            while (!client.available()) delay(1);
            String request = client.readStringUntil('\r');
            client.flush();
            int posX = request.indexOf("dx=");
            int posY = request.indexOf("dy=");
            int posType = request.indexOf("type=");
            // Vérifier si c'est une vraie requête de formulaire avec des paramètres
            bool isFormSubmit = false;
            int posSubmit = request.indexOf("submit=1");
            if (request.indexOf("GET /?sequence=") != -1 && posSubmit != -1) {
                isFormSubmit = true;
                addLog("[wifi] Formulaire de séquence carré recu");
            } else if (request.indexOf("GET /?cercle=") != -1 && posSubmit != -1) {
                isFormSubmit = true;
                addLog("[wifi] Formulaire de séquence cercle recu");
            } else if (request.indexOf("GET /?dx=") != -1 && posSubmit != -1) {
                isFormSubmit = true;
                addLog("[wifi] Formulaire de mouvement recu");
            } else if (request.indexOf("GET /?dx=") != -1 || request.indexOf("GET /?sequence=") != -1 || request.indexOf("GET /?cercle=") != -1 || request.indexOf("GET /?reset=") != -1) {
                // C'est un rechargement de page avec les paramètres dans l'URL
                addLog("[wifi] Recharge de page détectée, mouvement ignoré");
            }
            if (posX != -1 && posY != -1 && isFormSubmit) {
                bool isAbsoluteCoords = true;
                if (posType != -1) {
                    String typeValue = request.substring(posType + 5, request.indexOf('&', posType + 5));
                    if (typeValue == "robot") {
                        isAbsoluteCoords = false;
                        addLog("[wifi] Type de coordonnées: relatives au robot");
                    } else {
                        addLog("[wifi] Type de coordonnées: absolues");
                    }
                } else {
                    addLog("[wifi] Type non spécifié, utilisation des coordonnées absolues par défaut");
                }
                float dx = request.substring(posX + 3, request.indexOf('&', posX)).toFloat();
                float dy = request.substring(posY + 3).toFloat();
                addLog("[wifi] Valeurs reçues: dx=" + String(dx) + ", dy=" + String(dy));
                if (isAbsoluteCoords) {
                    addLog("[wifi] Conversion de coordonnées absolues vers robot");
                    DeltaXY absolutePoint(dx, dy);
                    DeltaXY robotCoord = convertAbsoluteToRobotCoordinates(absolutePoint, robotState);
                    demarer(robotCoord.x, robotCoord.y);
                } else {
                    addLog("[wifi] Utilisation directe des coordonnées relatives au robot");
                    demarer(dx, dy);
                }
            } else {
                int posSequence = request.indexOf("sequence=1");
                int posCercle = request.indexOf("cercle=1");
                int posReset = request.indexOf("reset=1");
                if (posReset != -1) {
                    addLog("[wifi] Détection paramètre reset=1 à la position " + String(posReset));
                    if (isFormSubmit) {
                        addLog("[wifi] Demande de réinitialisation confirmée");
                        resetRobot();
                        addLog("[wifi] Vérification après réinitialisation: X=" + String(robotState.x) + ", Y=" + String(robotState.y) + ", Theta=" + String(robotState.theta));
                    } else {
                        addLog("[wifi] Paramètre reset=1 détecté mais formulaire non soumis");
                    }
                } else if (posSequence != -1) {
                    addLog("[wifi] Détection paramètre sequence=1 à la position " + String(posSequence));
                    if (isFormSubmit) {
                        addLog("[wifi] Demande de démarrage de la séquence automatique confirmée");
                        if (deplacementFait && !sequenceEnCours && !sequenceCercleEnCours) {
                            sequenceEnCours = true;
                            etapeSequence = 0;
                            executerProchainMouvement = true;
                            addLog("[wifi] Séquence automatique lancée - variables: sequenceEnCours=" + String(sequenceEnCours) + ", etapeSequence=" + String(etapeSequence));
                            executerSequenceAutomatique();
                        } else {
                            addLog("[wifi] Impossible de démarrer la séquence, robot occupé - deplacementFait=" + String(deplacementFait) + ", sequenceEnCours=" + String(sequenceEnCours));
                        }
                    } else {
                        addLog("[wifi] Paramètre sequence=1 détecté mais formulaire non soumis");
                    }
                } else if (posCercle != -1) {
                    addLog("[wifi] Détection paramètre cercle=1 à la position " + String(posCercle));
                    if (isFormSubmit) {
                        addLog("[wifi] Demande de démarrage de la séquence cercle confirmée");
                        if (deplacementFait && !sequenceCercleEnCours && !sequenceEnCours) {
                            sequenceCercleEnCours = true;
                            etapeCercle = 0;
                            executerProchainPointCercle = true;
                            addLog("[wifi] Séquence cercle lancée - variables: sequenceCercleEnCours=" + String(sequenceCercleEnCours) + ", etapeCercle=" + String(etapeCercle));
                            executerSequenceCercle();
                        } else {
                            addLog("[wifi] Impossible de démarrer la séquence cercle, robot occupé - deplacementFait=" + String(deplacementFait) + ", sequenceCercleEnCours=" + String(sequenceCercleEnCours));
                        }
                    } else {
                        addLog("[wifi] Paramètre cercle=1 détecté mais formulaire non soumis");
                    }
                } else {
                    addLog("[wifi] Page chargée sans commande");
                }
            }
            String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Drawbot WiFi</title>";
            html += "<style>";
            html += "body{font-family:Arial;text-align:center;background:#f2f2f2;padding:20px;}";
            html += "form{background:#fff;padding:20px;border-radius:12px;box-shadow:0 0 10px #888;display:inline-block;margin-bottom:20px;width:80%;max-width:400px;}";
            html += "input{padding:10px;margin:8px;border-radius:5px;width:80%;}";
            html += "input[type=submit]{background:#4CAF50;color:white;border:none;cursor:pointer;}";
            html += "input[type=submit]:hover{background:#45a049;}";
            html += ".log-container{background:#fff;border-radius:12px;box-shadow:0 0 10px #888;padding:15px;margin:0 auto;width:90%;max-width:800px;text-align:left;max-height:400px;overflow-y:auto;}";
            html += ".log-entry{font-family:monospace;font-size:0.9em;margin:3px 0;border-bottom:1px solid #eee;padding-bottom:3px;}";
            html += "h2{color:#333;margin-top:25px;}";
            html += "</style>";
            html += "</head><body>";
            html += "<h1>Drawbot WiFi</h1>";
            html += "<div style='background:#f8f8f8;padding:5px;border-radius:5px;margin-bottom:10px;font-size:0.8em;text-align:right;'>";
            html += "Compilé le " + String(COMPILE_DATE) + " à " + String(COMPILE_TIME);
            html += "</div>";
            html += "<div style='background:#fff;padding:10px;border-radius:10px;margin-bottom:15px;'><strong>Position: </strong>";
            html += "X: " + String(robotState.x, 1) + " cm, ";
            html += "Y: " + String(robotState.y, 1) + " cm, ";
            html += "Angle: " + String(robotState.theta * 180.0 / PI, 1) + "° </div>";
            html += "<form action='/' method='get'>";
            html += "<h2>Commande</h2>";
            html += "<div class='input-group'>";
            html += "<label for='dx'>Delta X (cm):</label>";
            html += "<input type='number' step='0.1' name='dx' id='dx' value='0' required>";
            html += "</div>";
            html += "<div class='input-group'>";
            html += "<label for='dy'>Delta Y (cm):</label>";
            html += "<input type='number' step='0.1' name='dy' id='dy' value='0' required>";
            html += "</div>";
            html += "<div class='input-group'>";
            html += "<label for='type'>Type de coordonnées:</label>";
            html += "<select name='type' id='type'>";
            html += "<option value='absolu' selected>Absolues</option>";
            html += "<option value='robot'>Relatives au robot</option>";
            html += "</select>";
            html += "</div>";
            html += "<input type='hidden' name='submit' value='1'>";
            html += "<input type='submit' value='Déplacer'>";
            html += "</form>";
            html += "<div style='margin-top:20px; margin-bottom:20px;'>";
            html += "<h2>Déplacement rapide</h2>";
            html += "<p>Déplacement absolu de 0,1 cm</p>";
            html += "<div style='display:grid; grid-template-columns:1fr 1fr 1fr; max-width:180px; margin:0 auto; gap:5px;'>";
            html += "<div></div>";
            html += "<a href='/?dx=0&dy=0.1&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>&#8593;</a>";
            html += "<div></div>";
            html += "<a href='/?dx=-0.1&dy=0&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>&#8592;</a>";
            html += "<div style='background:#ddd; color:#666; padding:5px; border-radius:5px;'>+0,1</div>";
            html += "<a href='/?dx=0.1&dy=0&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>&#8594;</a>";
            html += "<div></div>";
            html += "<a href='/?dx=0&dy=-0.1&type=absolu&submit=1' style='background:#4CAF50; color:white; padding:10px; border-radius:5px; text-decoration:none;'>&#8595;</a>";
            html += "<div></div>";
            html += "</div>";
            html += "</div>";
            html += "<div style='margin-top:20px; margin-bottom:20px;'>";
            html += "<h2>Séquence Automatique</h2>";
            html += "<p>Dessiner un carré : 10 pas à droite, 10 pas en haut, 10 pas à droite</p>";
            html += "<a href='/?sequence=1&submit=1' style='background:#FF5722; color:white; padding:15px 30px; border-radius:5px; text-decoration:none; display:inline-block; margin:10px; font-weight:bold;'>Lancer la séquence</a>";
            html += sequenceEnCours ? "<p><strong>Séquence en cours : Étape " + String(etapeSequence) + "/" + String(ETAPES_SEQUENCE_MAX) + "</strong></p>" : "";
            html += "</div>";
            html += "<div style='margin-top:20px; margin-bottom:20px;'>";
            html += "<h2>Dessiner un Cercle</h2>";
            html += "<p>Cercle de 1 cm de diamètre en 100 points</p>";
            html += "<a href='/?cercle=1&submit=1' style='background:#2196F3; color:white; padding:15px 30px; border-radius:5px; text-decoration:none; display:inline-block; margin:10px; font-weight:bold;'>Dessiner le cercle</a>";
            html += sequenceCercleEnCours ? "<p><strong>Cercle en cours : Point " + String(etapeCercle) + "/" + String(ETAPES_CERCLE_MAX) + "</strong></p>" : "";
            html += "</div>";
            html += "<div style='margin-top:20px; margin-bottom:20px;'>";
            html += "<h2>Réinitialisation</h2>";
            html += "<p>Réinitialiser la position du robot à (0,0) et l'angle à 0°</p>";
            html += "<a href='/?reset=1&submit=1' style='background:#F44336; color:white; padding:15px 30px; border-radius:5px; text-decoration:none; display:inline-block; margin:10px; font-weight:bold;'>Réinitialiser position</a>";
            html += "<p><strong>Position actuelle: X=" + String(robotState.x, 2) + " cm, Y=" + String(robotState.y, 2) + " cm, Angle=" + String(robotState.theta * 180.0 / PI, 1) + "°</strong></p>";
            html += "</div>";
            html += "<h2>Logs</h2>";
            html += "<div class='log-container'>" + getAllLogs() + "</div>";
            html += "</body></html>";
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");
            client.println();
            client.println(html);
            delay(1);
            client.stop();
        }
    }
}
