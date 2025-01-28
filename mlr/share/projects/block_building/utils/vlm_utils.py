import base64
import puremagic


class OpenAIUtils:
    @staticmethod
    def convert_to_base64(img_path):
        with open(img_path, "rb") as img_file:
            return base64.b64encode(img_file.read()).decode("utf-8")

    @staticmethod
    def construct_message(dev_message=None, usr_message=None, asst_message=None):
        if dev_message:
            if isinstance(dev_message, list):
                return {
                    "role": "developer",
                    "content": dev_message
                }
            return {
                "role": "developer",
                "content": [dev_message]
            }

        if usr_message:
            if isinstance(usr_message, list):
                return {
                    "role": "user",
                    "content": usr_message
                }

            return {
                "role": "user",
                "content": [usr_message]
            }

        if asst_message:
            if isinstance(asst_message, list):
                return {
                    "role": "assistant",
                    "content": asst_message
                }
            return {
                "role": "assistant",
                "content": [asst_message]
            }

    @staticmethod
    def get_text_content(input_prompt):
        if isinstance(input_prompt, list):
            input_prompt = "".join(input_prompt)

        return {
            "type": "text",
            "text": input_prompt
        }

    @staticmethod
    def get_img_content(input_prompt=None, img_filename=None):
        return_content = []

        if input_prompt:
            if isinstance(input_prompt, list):
                input_prompt = "".join(input_prompt)

            return_content.append(
                {
                    "type": "text",
                    "text": input_prompt
                }
            )

        if img_filename:
            img_base64 = OpenAIUtils.convert_to_base64(img_filename)
            img_mime_type = puremagic.magic_file(img_filename)[0].mime_type
            return_content.append(
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:{img_mime_type};base64,{img_base64}"},
                }
            )

        return return_content
