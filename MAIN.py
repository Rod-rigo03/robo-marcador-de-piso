import numpy as np
import socket
import time
import re


#PARAMETROS ROBO
passos_por_volta = 1600
velocidade_max = 1600 #PASSOS POR SEGUNDO
distancia_caneta_roda_externa = 230 #MM
distancia_caneta_roda_interna = 130  #MM
largura_robo = 360 #MM
diametro_da_roda = 100 #MM
ri = 1500 #MM QUE TAMBEM É A CORDENADA Y

#PARAMETROS REDE
HOST = "192.168.4.1"
PORT = 8080

2

while True:
    print("\n=== MENU ===")
    print("1 - Ir até um ponto")
    print("2 - Fazer uma cruz")
    print("3 - Girar 90 graus")
    print("4 - Alinhar com o laser")
    print("5 - Sair")

    opcao = input("Escolha uma opção: ")

    match opcao:
        case "1":
            print("\nDigite a coordenada do ponto de destino")
            x = (float(input("Digite a coordenada X de destino em METROS: ").replace(",", ".")) * 1000)+0.000000000000000000000000000000001
            y = (float(input("Digite a coordenada Y de destino em METROS: ").replace(",", ".")) * 1000)+0.000000000000000000000000000000001

            ### laser
            ### colocar o codigo para ler coordenada do mouse no laser
            arquivo = r"D:\ROBO\INTERAÇÃO PYTHON X IDE ARDUINO\Medida_LT.txt"

            with open(arquivo, "r", encoding="utf-8") as f:
                texto = f.read()

            # encontra todos os números (incluindo vírgula ou ponto)
            numeros = re.findall(r"[-+]?\d*[,\.]?\d+", texto)

            # converte vírgulas decimais para ponto
            valores = [float(n.replace(",", ".")) for n in numeros]

            pos_inic_x = valores[0]*1000
            pos_inic_y = valores[1]*1000
            pos_inic_z = valores[2]*1000
            alpha      = valores[3]

            while True:
                
                cr = (2 * np.pi * diametro_da_roda) / 2
                origem_x = 0
                origem_y = 0
                
                # Calcular raios em mm

                raio_inicial = np.sqrt((pos_inic_x - origem_x)**2 + (pos_inic_y - origem_y)**2)
                raio_desejado = np.sqrt((x - origem_x)**2 + (y - origem_y)**2)

                # Calcular ângulos em radianos
                alpha_inicial = np.arctan2(pos_inic_y, pos_inic_x)
                alpha_desejado = np.arctan2(y, x)

                # Variação do raio
                variacao_raio = raio_desejado / raio_inicial

                # ----------------------------
                # Deslocamentos tangenciais em mm
                arco_externo_em_mm = (raio_inicial + distancia_caneta_roda_externa) * (alpha_desejado - alpha_inicial)
                arco_interno_em_mm = (raio_inicial - distancia_caneta_roda_interna) * (alpha_desejado - alpha_inicial)

                #tang

                dz = (arco_externo_em_mm/cr)*passos_por_volta
                dy = -((arco_interno_em_mm/cr)*passos_por_volta)
                vy = (dy * velocidade_max) / dz
                vz = (velocidade_max) 
                da = (((alpha_desejado - alpha_inicial) * (largura_robo/2))/cr)*passos_por_volta
                va = (da*velocidade_max)/dz
                vx = va
                dx = da

                #radial

                drx = ((raio_desejado - raio_inicial)/cr)*passos_por_volta
                dra = -drx
                vra = velocidade_max
                vrx = velocidade_max

              
                if variacao_raio > 1:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                    try:
                        print(f"Conectando ao ESP8266 em {HOST}:{PORT}...")
                        s.connect((HOST, PORT))
                        print("✅ Conexão estabelecida!")
                        
                        # Envia UMA ÚNICA VEZ
                        mensagem = f"VX={vx};DX={dx};VY={vy};DY={dy};VZ={vz};DZ={dz};VA={va};DA={da}"
                        s.sendall((mensagem + "\n").encode())
                        print(f"Enviado: {mensagem}")
                        
                        # Aguarda até receber "✅ Movimento concluído."
                        while True:
                            resposta = s.recv(1024).decode().strip()
                            if resposta:
                                print(f"⬅️ {resposta}")
                                if "concluído" in resposta:
                                    break
                        
                        print("✅ Movimento Tangencial finalizado!")
                        
                        time.sleep(2)

                        mensagem = f"VX={vrx};DX={drx};VY=0;DY=0;VZ=0;DZ=0;VA={vra};DA={dra}"
                        s.sendall((mensagem + "\n").encode())
                        print(f"📤 Enviado: {mensagem}")
                        
                        # Aguarda até receber "✅ Movimento concluído."
                        while True:
                            resposta = s.recv(1024).decode().strip()
                            if resposta:
                                print(f"⬅️ {resposta}")
                                if "concluído" in resposta:
                                    break
                                
                        
                        print("✅ Movimento finalizado!")

                        
                    except Exception as e:
                        print("❌ Erro:", e)
                    finally:
                        s.close()

                elif variacao_raio < 1:
                    
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                    try:
                        print(f"Conectando ao ESP8266 em {HOST}:{PORT}...")
                        s.connect((HOST, PORT))
                        print("✅ Conexão estabelecida!")
                        
                        # Envia UMA ÚNICA VEZ
                        mensagem = f"VX={vrx};DX={drx};VY=0;DY=0;VZ=0;DZ=0;VA={vra};DA={dra}"
                        
                        s.sendall((mensagem + "\n").encode())
                        print(f"📤 Enviado: {mensagem}")
                        
                        # Aguarda até receber "✅ Movimento concluído."
                        while True:
                            resposta = s.recv(1024).decode().strip()
                            if resposta:
                                print(f"⬅️ {resposta}")
                                if "concluído" in resposta:
                                    break
                        
                        print("✅ Movimento Radial finalizado!")
                        
                        time.sleep(2)

                        mensagem = f"VX={vx};DX={dx};VY={vy};DY={dy};VZ={vz};DZ={dz};VA={va};DA={da}"
                        s.sendall((mensagem + "\n").encode())
                        print(f"📤 Enviado: {mensagem}")
                        
                        # Aguarda até receber "✅ Movimento concluído."
                        while True:
                            resposta = s.recv(1024).decode().strip()
                            if resposta:
                                print(f"⬅️ {resposta}")
                                if "concluído" in resposta:
                                    break
                                
                        
                        print("✅ Movimento finalizado!")

                        
                    except Exception as e:
                        print("❌ Erro:", e)
                    finally:
                        s.close()

                else:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                    try:
                        print(f"Conectando ao ESP8266 em {HOST}:{PORT}...")
                        s.connect((HOST, PORT))
                        print("✅ Conexão estabelecida!")
                        
                        # Envia UMA ÚNICA VEZ
                        mensagem = f"VX={vx};DX={dx};VY={vy};DY={dy};VZ={vz};DZ={dz};VA={va};DA={da}"
                        s.sendall((mensagem + "\n").encode())
                        print(f"📤 Enviado: {mensagem}")
                        
                        # Aguarda até receber "✅ Movimento concluído."
                        while True:
                            resposta = s.recv(1024).decode().strip()
                            if resposta:
                                print(f"⬅️ {resposta}")
                                if "concluído" in resposta:
                                    break
                        
                        print("✅ Movimento finalizado!")
                        
                        time.sleep(2)

                    except Exception as e:
                        print("❌ Erro:", e)
                    finally:
                        s.close()
                        
                ##### laser
                ### colocar o codigo para ler coordenada do mouse no laser (time.sleep(2))
                arquivo2 = r"D:\ROBO\INTERAÇÃO PYTHON X IDE ARDUINO\Medida_LT_final.txt"

                with open(arquivo2, "r", encoding="utf-8") as f:
                    texto = f.read()

                # encontra todos os números (incluindo vírgula ou ponto)
                numeros = re.findall(r"[-+]?\d*[,\.]?\d+", texto)

                # converte vírgulas decimais para ponto
                valores = [float(n.replace(",", ".")) for n in numeros]

                pos_inic_x = valores[0]*1000
                pos_inic_y = valores[1]*1000
                pos_inic_z = valores[2]*1000
                alpha1      = valores[3]

                if abs(pos_inic_x - x) <= 5 and abs(pos_inic_y - y) <= 5:
                    print("\nO robo chegou ao destino")
                    break
                else:
                    print("\nO robo ainda não chegou ao destino")
                    print("Corrigindo...\n")
                    time.sleep(2)
                    continue

        case "2":
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                print(f"Conectando ao ESP8266 em {HOST}:{PORT}...")
                s.connect((HOST, PORT))
                print("✅ Conexão estabelecida!")
                
                # Envia UMA ÚNICA VEZ
                mensagem = "CRUZ"
                
                s.sendall((mensagem + "\n").encode())
                print(f"📤 Enviado: {mensagem}")
                
                # Aguarda até receber "✅ Movimento concluído."
                while True:
                    resposta = s.recv(1024).decode().strip()
                    if resposta:
                        print(f"⬅️ {resposta}")
                        if "concluído" in resposta:
                            break
            except Exception as e:
                    print("❌ Erro:", e)
            finally:
                s.close()
            continue

        case "3":
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            try:
                print(f"Conectando ao ESP8266 em {HOST}:{PORT}...")
                s.connect((HOST, PORT))
                print("✅ Conexão estabelecida!")
                
                # Envia UMA ÚNICA VEZ
                #mensagem = "VX=228;DX=-1440;VY=1142;DY=7200;VZ=1600;DZ=-10080;VA=228;DA=-1440"
                mensagem = "VX=228;DX=1440;VY=1142;DY=-7200;VZ=1600;DZ=10080;VA=228;DA=1440"
                s.sendall((mensagem + "\n").encode())
                print(f"📤 Enviado: {mensagem}")
                
                # Aguarda até receber "✅ Movimento concluído."
                while True:
                    resposta = s.recv(1024).decode().strip()
                    if resposta:
                        print(f"⬅️ {resposta}")
                        if "concluído" in resposta:
                            break
                
                print("✅ Movimento Tangencial finalizado!")
            except Exception as e:
                print("❌ Erro:", e)
            finally:
                s.close()
            continue
        case "4":
            print("\nEm desenvolvimento...")
            continue
        case "5":
            print("Saindo...")
            break
        case _:
            print("\nOpção inválida!")
            pass


