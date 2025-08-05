import json

def get_data(username: str) -> str:
    with open("personas.json", "r", encoding="utf-8") as f:
        personas = json.load(f)
    print("name: ", username)
    return personas.get(username.lower(), "Sorry, no persona data found.")


# print(get_data("steve_harris"))