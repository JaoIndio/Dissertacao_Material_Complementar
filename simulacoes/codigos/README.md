# Instruções para Executar Simulações

Siga essas instruções para configurar seu ambiente local e conseguir rodar os scripts de simulação

### Requisistos

- **Python 3.8+:** Garanta que sua máquina tenha o python instalado. Você pode verificar sua versão executando `python3 --version` no seu terminal.
- **Git:** Para clonar este repositório

Primeiramente, clone o repositório na sua máquina local:

```
git clone https://github.com/JaoIndio/Dissertacao_Material_Complementar/tree/master.git
cd Dissertacao_Material_Complementar
```

Navegue até o local onde os códigos estão presentes:

```
cd simulacoes/codigos
```

Crie um ambiente virtual python:

```
# Cria o ambiente virtual chamado 'venv'
python3 -m venv venv
```

Ative o ambiente virtual:

- **No Linux/macOS**

```
source venv/bin/activate
```

- **No Windows**

```
.\venv\Scripts\activate
```

  Com o seu ambiente virtual ativado, instale as bibliotecas e pacotes necessários utilizando o arquivo `requirements.txt`:

```
pip install --upgrade pip
pip install -r requirements.txt
```

  Rode um dos scripts:

```
python simulacaoDiodo.py
```

  **OU**

```
python simulacaoLED.py
```
